#include "drake/manipulation/conveyor_belt/conveyor_belt_velocity_controller.h"

#include "drake/math/saturate.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace conveyor_belt {

using Eigen::Vector2d;

using systems::BasicVector;

const int kNumJoints = 6;

ConveyorBeltPdController::ConveyorBeltPdController(){

  desired_state_input_port_ =
      this->DeclareVectorInputPort("desired_state", BasicVector<double>(2 * kNumJoints))
          .get_index();

  state_input_port_ =
      this->DeclareVectorInputPort("state", BasicVector<double>(2 * kNumJoints))
          .get_index(); 

  generalized_velocity_output_port_ =
      this->DeclareVectorOutputPort(
              "generalized_velocity", BasicVector<double>(kNumJoints),
              &ConveyorBeltPdController::CalcGeneralizedVelocityOutput)
          .get_index();

  this->set_name("conveyor_controller");
}

void ConveyorBeltPdController::CalcGeneralizedVelocityOutput(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output_vector) const {

    const auto& desired_state = get_desired_state_input_port().Eval(context);
    const auto& state = get_state_input_port().Eval(context);
    drake::log()->info(state[0]);

    // Set desired state to current position and input velocity
    output_vector->SetFromVector(Vector2d(state[0],desired_state[1]));
}

ConveyorBeltVelocityController::ConveyorBeltVelocityController(double time_step) {
  systems::DiagramBuilder<double> builder;
  auto pd_controller = builder.AddSystem<ConveyorBeltPdController>();
  state_interpolator_ =
      builder.AddSystem<systems::StateInterpolatorWithDiscreteDerivative>(
          1, time_step);

  builder.Connect(state_interpolator_->get_output_port(),
                  pd_controller->get_desired_state_input_port());
                  
  desired_position_input_port_ = 
      builder.ExportInput(state_interpolator_->get_input_port(), "desired_velocity");

  state_input_port_ =
      builder.ExportInput(pd_controller->get_state_input_port(), "state");

  generalized_velocity_output_port_ = 
      builder.ExportOutput(pd_controller->get_generalized_velocity_output_port(), "generalized_velocity");

  builder.BuildInto(this);
}

void ConveyorBeltVelocityController::set_initial_position(
    drake::systems::State<double>* state, double desired_position) const {
  state_interpolator_->set_initial_position(
      &this->GetMutableSubsystemState(*state_interpolator_, state),
      Vector1d(desired_position));
}

}  // namespace ConveyorBelt
}  // namespace manipulation
}  // namespace drake
