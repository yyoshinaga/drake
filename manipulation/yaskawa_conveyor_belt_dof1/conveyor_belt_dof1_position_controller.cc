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

  // Desired states include:
  // 1 set of whisker position,
  // 1 set of whisker velocity,
  // 1 pusher position,
  // 1 pusher velocity, 
  // 1 puller position,
  // 1 pusher velocity
  desired_state_input_port_ =
      this->DeclareVectorInputPort("desired_state", BasicVector<double>(6))
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
  generalized_actuation_output_port_ =
      this->DeclareVectorOutputPort(
            "generalized_actuation", BasicVector<double>(4),
            &EndEffectorPdController::CalcGeneralizedActuationOutput)
          .get_index();    

  this->set_name("end_effector_controller");
}

//This is where you calculate accelerations for ee input
Vector4d EndEffectorPdController::CalcGeneralizedActuation(
    const drake::systems::Context<double>& context) const {
          drake::log()->info("calcgeneralizedactuation begin");

  // Read the input ports.
  const auto& desired_state = get_desired_state_input_port().Eval(context);
  // TODO(russt): Declare a proper input constraint.
  const auto& state = get_state_input_port().Eval(context);

  // f₀+f₁ = -kp_constraint*(q₀+q₁) - kd_constraint*(v₀+v₁).
  const double f0_plus_f1 = -kp_constraint_ * (state[0] + state[1]) -
                            kd_constraint_ * (state[4] + state[5]);

  // -f₀+f₁ = sat(kp_command*(q_d + q₀ - q₁) + kd_command*(v_d + v₀ - v₁)).
  const double neg_f0_plus_f1 =
                        kp_command_ * (desired_state[0] + state[0] - state[1]) +
                        kd_command_ * (desired_state[3] + state[4] - state[5]);


  //TODO: Use received status to control ee pusher and puller accelerations

  drake::log()->info("calcgeneralizedactuation end");

  // f₀ = (f₀+f₁)/2 - (-f₀+f₁)/2,
  // f₁ = (f₀+f₁)/2 + (-f₀+f₁)/2.
  return Vector4d(0.5 * f0_plus_f1 - 0.5 * neg_f0_plus_f1,
                  0.5 * f0_plus_f1 + 0.5 * neg_f0_plus_f1,
                  0,
                  0.1 );
}

//actuations/acceleration for each component
void EndEffectorPdController::CalcGeneralizedActuationOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {

  drake::log()->info("calcgeneralizedactuationOutput begin");
  output_vector->SetFromVector(CalcGeneralizedActuation(context));
  drake::log()->info("calcgeneralizedActuationOutput end");
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

  state_input_port_ = builder.ExportInput(
      pd_controller->get_state_input_port(), "state");

  generalized_actuation_output_port_ = builder.ExportOutput(
      pd_controller->get_generalized_actuation_output_port(), "generalized_actuation");

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
