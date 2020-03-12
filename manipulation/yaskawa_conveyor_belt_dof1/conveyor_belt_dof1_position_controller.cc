#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_position_controller.h"

#include "drake/math/saturate.h"
#include "drake/systems/framework/diagram_builder.h"
#include <gflags/gflags.h>

namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

DEFINE_double(angle_limit, M_PI/2, "movement limit");
DEFINE_double(pusher_limit, 0.7, "movement limit");
DEFINE_double(puller_limit, -0.3, "movement limit");
DEFINE_double(pusher_initial, 0.0, "initial position");
DEFINE_double(puller_initial, 0, "initial position");
DEFINE_double(belt_speed, 0.1, "speed limit");


using Eigen::Vector2d;
using Eigen::Vector4d;
using systems::BasicVector;

const int kNumJoints = 4;

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

  this->DeclareDiscreteState(1);    // ee states - where to put the pusher and puller
  this->DeclarePeriodicDiscreteUpdateEvent(0.0025, 0.0, &EndEffectorPdController::Update);


  // Desired states include:
  // 1 set of whisker position,
  // 1 set of whisker velocity,
  desired_whisker_state_input_port_ =
      this->DeclareVectorInputPort("desired_whisker_state", BasicVector<double>(2))
          .get_index();

  // Desired states include:
  // 1 pusher velocity, 
  // 1 pusher velocity
  desired_belt_state_input_port_ =
      this->DeclareVectorInputPort("desired_belt_state", BasicVector<double>(2))
          .get_index();

  // Input includes 8 states:
  // pos and vel for right whisker,
  // pos and vel for left whisker,
  // pos and vel for pusher, 
  // pos and vel for puller
  state_input_port_ =
      this->DeclareVectorInputPort("state", BasicVector<double>(2 * kNumJoints))
          .get_index();

  // Output vector is size 4
  // actuation for right whisker
  // actuation for left whisker
  // acc for pusher
  // acc for puller
  generalized_acceleration_output_port_ =
      this->DeclareVectorOutputPort(
            "generalized_acceleration", BasicVector<double>(4),
            &EndEffectorPdController::CalcGeneralizedAccelerationOutput)
          .get_index();    

  this->set_name("end_effector_controller");
}

void EndEffectorPdController::Update(const systems::Context<double>& context,
                                    systems::DiscreteValues<double>* updates) const {

    const double ee_state = context.get_discrete_state()[0];

    // Get continuous states from the plant
    const auto& desired_belt_state = get_desired_belt_state_input_port().Eval(context);
    
    const double pusher_state = desired_belt_state[0];
    const double puller_state = desired_belt_state[1];

    if(pusher_state == 0 && puller_state == 1 && ee_state == 3){
        //Move to Belt
        (*updates)[0] = 0;
    }
    else if(pusher_state == 0 && puller_state == 0 && ee_state == 0){
        //Before Collect
        (*updates)[0] = 1;
    }
    else if(pusher_state == 0 && puller_state == 1 && ee_state == 1){
        //After Collect/Carry 
        (*updates)[0] = 2;
    }
    else if(pusher_state == 1.0 && ee_state == 2){
        //Pushing to shelf
        (*updates)[0] = 3;
    }
    else{
        drake::log()->info("remain belt state cycle:\n {}\n {}\n {}",pusher_state,puller_state,ee_state);
    }
}

//This is where you calculate accelerations for ee input
Vector4d EndEffectorPdController::CalcGeneralizedAcceleration(
    const drake::systems::Context<double>& context) const {
    
  // Read the input ports.
  const auto& desired_whisker_state = get_desired_whisker_state_input_port().Eval(context);
  const auto& desired_belt_state = get_desired_belt_state_input_port().Eval(context);

  // TODO(russt): Declare a proper input constraint.
  const auto& state = get_state_input_port().Eval(context);

  // f₀+f₁ = -kp_constraint*(q₀+q₁) - kd_constraint*(v₀+v₁).
  const double f0_plus_f1 = -kp_constraint_ * (state[0] + state[1]) -
                            kd_constraint_ * (state[4] + state[5]);

  // -f₀+f₁ = sat(kp_command*(q_d + q₀ - q₁) + kd_command*(v_d + v₀ - v₁)).
  const double neg_f0_plus_f1 =
                        kp_command_ * (desired_whisker_state[0] + state[0] - state[1]) +
                        kd_command_ * (desired_whisker_state[1] + state[4] - state[5]);

  // state[2] = pusher position
  // state[3] = puller position
  // state[6] = pusher velocity
  // state[7] = puller velocity

  //Use desired velocities from belt_state to create accelerations
  // desired_belt_states are pusher and puller's velocities
//   const double kp = 50;
//   const double kd = 30;

  const double kp_2 = 1000;
  const double kd_2 = 500;

  double pusher_acc = 0; 
  double puller_acc = 0;
  const double buffer = 0.01;    
//   const double ee_state = context.get_discrete_state()[0];

  drake::log()->info("desired belt state:\n {}\n {}",desired_belt_state[0],desired_belt_state[1]);

    drake::log()->info("\nstate[2]: {}\nstate[3]: {}",state[2],state[3]);

  DRAKE_DEMAND(FLAGS_pusher_limit+buffer > state[2] && FLAGS_pusher_initial-buffer < state[2]);
  DRAKE_DEMAND(FLAGS_puller_limit-buffer < state[3] && FLAGS_puller_initial+buffer > state[3]);


  // If pusher is on,
  if(desired_belt_state[0]){
    drake::log()->info("pusher on\npos at: {}",state[2]);
    pusher_acc = kp_2*(FLAGS_pusher_limit - state[2]) + kd_2*(- state[6]);
    drake::log()->info("pusher is pushing");
  }
  // If pusher is off
  else{
    // If pusher is at its initial position
    drake::log()->info("pusher on\npos at: {}",state[2]);
    pusher_acc = kp_2*(FLAGS_pusher_initial - state[2]) + kd_2*( - state[6]);
    drake::log()->info("pusher is already at initial position");
  }

  // If puller is on, 
  if(desired_belt_state[1]){
    drake::log()->info("puller on\npos at: {}",state[3]);
    puller_acc = kp_2*(FLAGS_puller_limit - state[3]) + kd_2*( - state[7]);
    drake::log()->info("puller already past its limit {}",state[3]);
  }
  // If puller is off
  else{
    drake::log()->info("puller on\npos at: {}",state[3]);
    puller_acc = kp_2*(FLAGS_puller_initial - state[3]) + kd_2*(- state[7]);
    drake::log()->info("puller is already at initial position");
  }

//   // If pusher and puller are both off, go to maintain current state
//   else{
//     drake::log()->info("go back to initial position for both pusher and puller");
//     pusher_acc = kp_2*(FLAGS_pusher_initial - state[2]) + kd_2*(- state[6]);
//     puller_acc = kp_2*(FLAGS_puller_initial - state[3]) + kd_2*(- state[7]);
//   }

//   drake::log()->info("\ndesired_whisker_state: {}\nstate[0]: {} \nstate[1]: {}\nstate[2]: {}\nstate[3]: {}\nstate[4]: {}\nstate[5]: {}\nstate[6]: {}\nstate[7]: {}\ndesired_belt_state[0]: {}\ndesired_belt_state[1]: {}\npusher_acc: {}\npuller_acc: {}", 
//   desired_whisker_state[0],state[0],state[1],state[2],state[3],state[4],state[5],state[6],state[7],desired_belt_state[0],desired_belt_state[1],pusher_acc,puller_acc);
    drake::log()->info("\npusher acc: {}\npuller acc: {}",pusher_acc,puller_acc);
    drake::log()->info("\npusher pos: {}\npuller pos: {}",state[2],state[3]);

  // f₀ = (f₀+f₁)/2 - (-f₀+f₁)/2,
  // f₁ = (f₀+f₁)/2 + (-f₀+f₁)/2.
  return Vector4d(0.5 * f0_plus_f1 - 0.5 * neg_f0_plus_f1,
                  0.5 * f0_plus_f1 + 0.5 * neg_f0_plus_f1,
                  pusher_acc,
                  puller_acc);
}

//actuations/acceleration for each component
void EndEffectorPdController::CalcGeneralizedAccelerationOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {

    output_vector->SetFromVector(CalcGeneralizedAcceleration(context));
}


EndEffectorGenerateAngleAndVelocity::EndEffectorGenerateAngleAndVelocity(){
  actuation_input_port_ =
      this->DeclareVectorInputPort("actuation", BasicVector<double>(3))
          .get_index();

  angle_output_port_ =
      this->DeclareVectorOutputPort(
            "angle", BasicVector<double>(1),
            &EndEffectorGenerateAngleAndVelocity::CalcAngleOutput)
          .get_index();    

  velocity_output_port_ =
      this->DeclareVectorOutputPort(
            "velocity", BasicVector<double>(2),
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
        output_vector->SetAtIndex(0, M_PI/2);
    }
    else{
        //Return whiskers to initial positions
        output_vector->SetAtIndex(0, 0);
    }
}

//Input: boolean on/off for moving pusher and puller
//Output: Velocity for pusher and puller
void EndEffectorGenerateAngleAndVelocity::CalcVelocityOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {

    const auto& actuation = get_actuation_input_port().Eval(context);
    
    // If both pusher and puller is on, something has gone wrong.
    // AKA: conveyor belt cannot push and pull at the same time
    // DRAKE_DEMAND(!(actuation[1] && actuation[2]));

    // output_vector->SetAtIndex(0, FLAGS_belt_speed); 
    // output_vector->SetAtIndex(1, -FLAGS_belt_speed);
    drake::log()->info("actuation input port:\n 0:{}\n 1:{}\n 2:{}",actuation[0],actuation[1],actuation[2]);

    //If pusher or puller is 'on', set velocity to desired velocity
    if(actuation[1]){
        output_vector->SetAtIndex(0, FLAGS_belt_speed); 
        output_vector->SetAtIndex(1, 0);
    }
    else if(actuation[2]){
        output_vector->SetAtIndex(0, 0);
        output_vector->SetAtIndex(1, -FLAGS_belt_speed); 
    }
    else{ 
        //If both are false, conveyor belt goes back to initial position
        output_vector->SetAtIndex(0, 0); 
        output_vector->SetAtIndex(1, 0); 
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

  builder.Connect(angle_and_velocity_generator->get_angle_output_port(),
                  state_interpolator_->get_input_port());

  // For whiskers
  builder.Connect(state_interpolator_->get_output_port(),
                  pd_controller->get_desired_whisker_state_input_port());

  // For pusher and puller
  builder.Connect(angle_and_velocity_generator->get_velocity_output_port(),
                  pd_controller->get_desired_belt_state_input_port());

  state_input_port_ = builder.ExportInput(
      pd_controller->get_state_input_port(), "state");

  generalized_acceleration_output_port_ = builder.ExportOutput(
      pd_controller->get_generalized_acceleration_output_port(), "generalized_acceleration");

  builder.BuildInto(this);
}

void EndEffectorPositionController::set_initial_position(
    drake::systems::State<double>* state, const Eigen::Ref<const Vector3<double>> desired_position) const {
  state_interpolator_->set_initial_position(
      &this->GetMutableSubsystemState(*state_interpolator_, state), desired_position);
}

}  // namespace yaskawa_conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
