#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_position_controller.h"

#include "drake/math/saturate.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

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

  // Desired states include: 1 set of whisker position, 1 set of whisker force, 1 pusher position, 1 pusher velocity, 
  // 1 puller position, 1 pusher velocity
  desired_state_input_port_ =
      this->DeclareVectorInputPort("desired_state", BasicVector<double>(6))
          .get_index();

  // Perhaps force limit should only apply to the whiskers
  force_limit_input_port_ =
      this->DeclareVectorInputPort("force_limit", BasicVector<double>(1))
          .get_index();

  // Input is the state of the system. 2 for right, 2 for left whisker, 2 pusher, 2 puller
  state_input_port_ =
      this->DeclareVectorInputPort("state", BasicVector<double>(2 * kNumJoints))
          .get_index();

  // Output is 2 for whisker
  generalized_force_output_port_ =
      this->DeclareVectorOutputPort(
              "generalized_force", BasicVector<double>(4),
              &EndEffectorPdController::CalcGeneralizedForceOutput)
          .get_index();

  //The status to send to the controller - need for 1 set of whiskers
  grip_force_output_port_ =
      this->DeclareVectorOutputPort("grip_force", BasicVector<double>(1),
                                    &EndEffectorPdController::CalcGripForceOutput)
          .get_index();

  // Accelerations of x_pos and theta is sent to multibody plants (mbp). Reason is because mbp doesn't want velocities
  pose_output_port_ = 
      this->DeclareVectorOutputPort("pose_output", systems::BasicVector<double>(2), 
                                &EndEffectorPdController::CalcPoseOutput).get_index();    //3?       

  this->set_name("end_effector_controller");
}

Vector4d EndEffectorPdController::CalcGeneralizedForce(
    const drake::systems::Context<double>& context) const {
          drake::log()->info("calcgeneralizedforce begin");

  // Read the input ports.
  const auto& desired_state = get_desired_state_input_port().Eval(context);
  const double force_limit = get_force_limit_input_port().Eval(context)[0];
  // TODO(russt): Declare a proper input constraint.
  DRAKE_DEMAND(force_limit > 0);
  const auto& state = get_state_input_port().Eval(context);

  // f₀+f₁ = -kp_constraint*(q₀+q₁) - kd_constraint*(v₀+v₁).
  const double f0_plus_f1 = -kp_constraint_ * (state[0] + state[1]) -
                            kd_constraint_ * (state[4] + state[5]);

  // -f₀+f₁ = sat(kp_command*(q_d + q₀ - q₁) + kd_command*(v_d + v₀ - v₁)).
  const double neg_f0_plus_f1 =
      math::saturate(kp_command_ * (desired_state[0] + state[0] - state[1]) +
                         kd_command_ * (desired_state[3] + state[4] - state[5]),
                     -force_limit, force_limit);

  drake::log()->info("calcgeneralizedforce end");


  // f₀ = (f₀+f₁)/2 - (-f₀+f₁)/2,
  // f₁ = (f₀+f₁)/2 + (-f₀+f₁)/2.
  return Vector4d(0.5 * f0_plus_f1 - 0.5 * neg_f0_plus_f1,
                  0.5 * f0_plus_f1 + 0.5 * neg_f0_plus_f1,0,0 );
}

void EndEffectorPdController::CalcGeneralizedForceOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {

  drake::log()->info("calcgeneralizedForceOutput begin");
  output_vector->SetFromVector(CalcGeneralizedForce(context));
  drake::log()->info("calcgeneralizedForceOutput end");
}

void EndEffectorPdController::CalcGripForceOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {

  drake::log()->info("CalcGripForceOutput begin");

  Vector4d force = CalcGeneralizedForce(context);
  output_vector->SetAtIndex(0, std::abs(force[0] - force[1]));


  drake::log()->info("CalcGripForceOutput end");

}

void EndEffectorPdController::CalcPoseOutput(const systems::Context<double>& context ,
                                            systems::BasicVector<double>* output) const {

    // Get the states from the plant
    const systems::BasicVector<double>* input_vector =
        this->EvalVectorInput(context, 0);
    auto inputVector = input_vector->CopyToVector();

    output->SetAtIndex(0,0);
    output->SetAtIndex(1,0);
                                        
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
          3, time_step);

  builder.Connect(state_interpolator_->get_output_port(),
                  pd_controller->get_desired_state_input_port());

  desired_position_input_port_ = builder.ExportInput(
      state_interpolator_->get_input_port(), "desired_position");

  force_limit_input_port_ = builder.ExportInput(
      pd_controller->get_force_limit_input_port(), "force_limit");

  state_input_port_ =
      builder.ExportInput(pd_controller->get_state_input_port(), "state");

  generalized_force_output_port_ = builder.ExportOutput(
      pd_controller->get_generalized_force_output_port(), "generalized_force");

  grip_force_output_port_ = builder.ExportOutput(
      pd_controller->get_grip_force_output_port(), "grip_force");

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
