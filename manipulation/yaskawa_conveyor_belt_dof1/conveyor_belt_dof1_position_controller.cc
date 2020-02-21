#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_position_controller.h"

#include "drake/math/saturate.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

using Eigen::Vector2d;

using systems::BasicVector;

const int kNumJoints = 2;

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

  desired_state_input_port_ =
      this->DeclareVectorInputPort("desired_state", BasicVector<double>(2))
          .get_index();
  force_limit_input_port_ =
      this->DeclareVectorInputPort("force_limit", BasicVector<double>(1))
          .get_index();
  state_input_port_ =
      this->DeclareVectorInputPort("state", BasicVector<double>(2 * kNumJoints))
          .get_index();

  generalized_force_output_port_ =
      this->DeclareVectorOutputPort(
              "generalized_force", BasicVector<double>(kNumJoints),
              &EndEffectorPdController::CalcGeneralizedForceOutput)
          .get_index();

  grip_force_output_port_ =
      this->DeclareVectorOutputPort("grip_force", BasicVector<double>(1),
                                    &EndEffectorPdController::CalcGripForceOutput)
          .get_index();

  this->set_name("end_effector_controller");
}

Vector2d EndEffectorPdController::CalcGeneralizedForce(
    const drake::systems::Context<double>& context) const {
  // Read the input ports.
  const auto& desired_state = get_desired_state_input_port().Eval(context);
  const double force_limit = get_force_limit_input_port().Eval(context)[0];
  // TODO(russt): Declare a proper input constraint.
  DRAKE_DEMAND(force_limit > 0);
  const auto& state = get_state_input_port().Eval(context);

  // f₀+f₁ = -kp_constraint*(q₀+q₁) - kd_constraint*(v₀+v₁).
  const double f0_plus_f1 = -kp_constraint_ * (state[0] + state[1]) -
                            kd_constraint_ * (state[2] + state[3]);

  // -f₀+f₁ = sat(kp_command*(q_d + q₀ - q₁) + kd_command*(v_d + v₀ - v₁)).
  const double neg_f0_plus_f1 =
      math::saturate(kp_command_ * (desired_state[0] + state[0] - state[1]) +
                         kd_command_ * (desired_state[1] + state[2] - state[3]),
                     -force_limit, force_limit);

  // f₀ = (f₀+f₁)/2 - (-f₀+f₁)/2,
  // f₁ = (f₀+f₁)/2 + (-f₀+f₁)/2.
  return Vector2d(0.5 * f0_plus_f1 - 0.5 * neg_f0_plus_f1,
                  0.5 * f0_plus_f1 + 0.5 * neg_f0_plus_f1);
}

void EndEffectorPdController::CalcGeneralizedForceOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {
  output_vector->SetFromVector(CalcGeneralizedForce(context));
}

void EndEffectorPdController::CalcGripForceOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {
  Vector2d force = CalcGeneralizedForce(context);
  output_vector->SetAtIndex(0, std::abs(force[0] - force[1]));
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
    drake::systems::State<double>* state, double desired_position) const {
  state_interpolator_->set_initial_position(
      &this->GetMutableSubsystemState(*state_interpolator_, state),
      Vector1d(desired_position));
}

}  // namespace yaskawa_conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake