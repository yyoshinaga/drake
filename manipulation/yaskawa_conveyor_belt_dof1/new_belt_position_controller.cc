#include "drake/manipulation/yaskawa_conveyor_belt_dof1/new_belt_position_controller.h"

#include "drake/math/saturate.h"
#include "drake/systems/framework/diagram_builder.h"
#include <gflags/gflags.h>

namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

DEFINE_double(angle_limit, M_PI, "movement limit");
DEFINE_double(belt_speed, 5, "speed limit");


using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using systems::BasicVector;

const int kNumWhiskers = 2;
const int kNumRollers = 7;

EndEffectorPdController::EndEffectorPdController(double kp_command,
                                             double kd_command,
                                             double kp_constraint,
                                             double kd_constraint)
    : kp_command_(kp_command),
      kd_command_(kd_command),
      kp_constraint_(kp_constraint),
      kd_constraint_(kd_constraint) {
  DRAKE_DEMAND(kp_command >= 0);
  DRAKE_DEMAND(kd_command >= 0);
  DRAKE_DEMAND(kp_constraint >= 0);
  DRAKE_DEMAND(kd_constraint >= 0);

  // Desired input states include:
  // 1 set of whisker position,
  // 1 set of whisker velocity,
  desired_whisker_state_input_port_ =
      this->DeclareVectorInputPort("desired_whisker_state", BasicVector<double>(2))
          .get_index();

  // Desired input states include:
  // 7 roller velocity
  desired_roller_state_input_port_ =
      this->DeclareVectorInputPort("desired_roller_state", BasicVector<double>(kNumRollers))
          .get_index();

  // Input includes 18 states?:
  // pos and vel for right whisker,
  // pos and vel for left whisker,
  // pos and vel for x amount of rollers, 
  state_input_port_ =
      this->DeclareVectorInputPort("state", BasicVector<double>(2*kNumWhiskers + 2*kNumRollers))
          .get_index();

  // Output vector is size 9
  // actuation for right whisker
  // actuation for left whisker
  // acc for roller
  generalized_acceleration_output_port_ =
      this->DeclareVectorOutputPort(
            "generalized_acceleration", BasicVector<double>(9),
            &EndEffectorPdController::CalcGeneralizedAccelerationOutput)
          .get_index();    

  this->set_name("end_effector_controller");
}

//This is where you calculate accelerations for ee input
VectorXd EndEffectorPdController::CalcGeneralizedAcceleration(
    const drake::systems::Context<double>& context) const {
    
  // Read the input ports.
  const auto& desired_whisker_state = get_desired_whisker_state_input_port().Eval(context);
  const auto& desired_roller_state = get_desired_roller_state_input_port().Eval(context);

  // TODO(russt): Declare a proper input constraint.
  const auto& state = get_state_input_port().Eval(context);

  // f₀+f₁ = -kp_constraint*(q₀+q₁) - kd_constraint*(v₀+v₁).
  const double f0_plus_f1 = -kp_constraint_ * (state[0] + state[1]) -
                            kd_constraint_ * (state[2+kNumRollers] + state[3+kNumRollers]);

  // -f₀+f₁ = sat(kp_command*(q_d + q₀ - q₁) + kd_command*(v_d + v₀ - v₁)).
  const double neg_f0_plus_f1 =
                        kp_command_ * (desired_whisker_state[0] + state[0] - state[1]) +
                        kd_command_ * (desired_whisker_state[1] + state[2+kNumRollers] - state[3+kNumRollers]);

  // Use desired velocities from roller_state to create accelerations
  const double kd = 10;
  double roller_acc = 0; 

  // If roller is on forwards,
  if(desired_roller_state[0] > 0){
    roller_acc = kd*(FLAGS_belt_speed - state[2*kNumWhiskers+kNumRollers]);
    drake::log()->info("Roller is moving forwards... trying to empty out");
  }
  // If roller is on backwards
  else if(desired_roller_state[0] < 0){
    roller_acc = kd*(-FLAGS_belt_speed - state[2*kNumWhiskers+kNumRollers]);
    drake::log()->info("Roller is moving backwards... trying to retrieve");
  }
  // If roller is off
  else{
    roller_acc = kd*(0 - state[2*kNumWhiskers+kNumRollers]);
    drake::log()->info("Roller is not moving");
  }

  for(int i = 0; i < 2*kNumWhiskers+2*kNumRollers; i++){
    drake::log()->info("state[{}]: {} ", i, state[i]);
  }

  drake::log()->info("roller_acc: {}", roller_acc);
  drake::log()->info("finger1_acc: {}", 0.5 * f0_plus_f1 - 0.5 * neg_f0_plus_f1);
  drake::log()->info("finger2_acc: {}", 0.5 * f0_plus_f1 + 0.5 * neg_f0_plus_f1);
  drake::log()->info("desired roller state: {}", desired_roller_state[0]);

  // f₀ = (f₀+f₁)/2 - (-f₀+f₁)/2,
  // f₁ = (f₀+f₁)/2 + (-f₀+f₁)/2.
  VectorXd actuator_efforts(9);
  actuator_efforts << 0.5 * f0_plus_f1 - 0.5 * neg_f0_plus_f1,
                  0.5 * f0_plus_f1 + 0.5 * neg_f0_plus_f1,
                  roller_acc, roller_acc, roller_acc, roller_acc, roller_acc, roller_acc, roller_acc;
  return actuator_efforts;
}

//actuations/acceleration for each component
void EndEffectorPdController::CalcGeneralizedAccelerationOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {

    output_vector->SetFromVector(CalcGeneralizedAcceleration(context));
}

EndEffectorGenerateAngleAndVelocity::EndEffectorGenerateAngleAndVelocity(){
  actuation_input_port_ =
      this->DeclareVectorInputPort("actuation", BasicVector<double>(2))
          .get_index();

  //This is for whiskers only
  angle_output_port_ =
      this->DeclareVectorOutputPort(
            "angle", BasicVector<double>(1),
            &EndEffectorGenerateAngleAndVelocity::CalcAngleOutput)
          .get_index();    

  //This is for roller only
  velocity_output_port_ =
      this->DeclareVectorOutputPort(
            "velocity", BasicVector<double>(7),
            &EndEffectorGenerateAngleAndVelocity::CalcVelocityOutput)
          .get_index(); 

  this->set_name("end_effector_angle_and_velocity_generator");
}

//Input: boolean on/off for moving whiskers
//Output: Whiskers desired angle
void EndEffectorGenerateAngleAndVelocity::CalcAngleOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {

    const auto& actuation = get_actuation_input_port().Eval(context);

    if(actuation[0]){
        //Move whiskers to grasping angle
        output_vector->SetAtIndex(0, FLAGS_angle_limit);
    }
    else{
        //Return whiskers to initial positions
        output_vector->SetAtIndex(0, 0);
    }
}

//Input: boolean forward/reverse/off for moving roller
//Output: Velocity for roller
void EndEffectorGenerateAngleAndVelocity::CalcVelocityOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {

    const auto& actuation = get_actuation_input_port().Eval(context);
    
    drake::log()->info("actuation[0]: {}", actuation[0]);
    drake::log()->info("actuation[1]: {}", actuation[1]);
    drake::log()->info("actuation[2]: {}", actuation[2]);
    drake::log()->info("actuation size: {}", actuation.size());

    // If roller is 'on', set velocity to desired velocity
    if(actuation[1] > 0){
        // output_vector->SetAtIndex(0, FLAGS_belt_speed); 
      for(int i = 0; i < 7; i++){
        output_vector->SetAtIndex(i, FLAGS_belt_speed);
      }
    }
    else if(actuation[1] < 0){
      for(int i = 0; i < 7; i++){
        output_vector->SetAtIndex(i, -FLAGS_belt_speed);
      }
    }
    else{ 

      for(int i = 0; i < 7; i++){
        output_vector->SetAtIndex(i, 0);
      }
        // output_vector->SetAtIndex(0, 0); 
    }
}

EndEffectorPositionController::EndEffectorPositionController(double time_step,
                                                            double kp_command,
                                                            double kd_command,
                                                            double kp_constraint,
                                                            double kd_constraint) {
  systems::DiagramBuilder<double> builder;

  auto pd_controller = builder.AddSystem<EndEffectorPdController>(
        kp_command, kd_command, kp_constraint, kd_constraint);

  state_interpolator_ =
      builder.AddSystem<systems::StateInterpolatorWithDiscreteDerivative>(
        1, time_step);

  auto angle_and_velocity_generator = 
        builder.AddSystem<EndEffectorGenerateAngleAndVelocity>();

  actuation_input_port_ = builder.ExportInput(
      angle_and_velocity_generator->get_actuation_input_port(), "ee_angle_and_velocity");


  // For whisker angle 
  builder.Connect(angle_and_velocity_generator->get_angle_output_port(),
                  state_interpolator_->get_input_port());
  builder.Connect(state_interpolator_->get_output_port(),
                  pd_controller->get_desired_whisker_state_input_port());

  // For roller velocity
  builder.Connect(angle_and_velocity_generator->get_velocity_output_port(),
                  pd_controller->get_desired_roller_state_input_port());

  state_input_port_ = builder.ExportInput(
      pd_controller->get_state_input_port(), "state");

  generalized_acceleration_output_port_ = builder.ExportOutput(
      pd_controller->get_generalized_acceleration_output_port(), "generalized_acceleration");

  builder.BuildInto(this);
}

void EndEffectorPositionController::set_initial_position(
    drake::systems::State<double>* state, const Eigen::Ref<const Vector2<double>> desired_position) const {
  state_interpolator_->set_initial_position(
      &this->GetMutableSubsystemState(*state_interpolator_, state), desired_position);
}

}  // namespace yaskawa_conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
