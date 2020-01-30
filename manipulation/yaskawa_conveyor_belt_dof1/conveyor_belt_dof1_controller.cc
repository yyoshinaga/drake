#include "drake/manipulation/conveyor_belt_dof1/conveyor_belt_dof1_controller.h"

#include "drake/manipulation/conveyor_belt_dof1/conveyor_belt_dof1_constants.h"
#include "drake/manipulation/conveyor_belt_dof1/conveyor_belt_dof1_lcm.h"
#include "drake/manipulation/conveyor_belt_dof1/conveyor_belt_dof1_plain_controller.h"
#include "drake/manipulation/conveyor_belt_dof1/conveyor_belt_dof1_trajectory_generator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

ConveyorBeltController::ConveyorBeltController(double kp, double ki, double kd) {
  systems::DiagramBuilder<double> builder;

  auto state_pass_through = builder.AddSystem<systems::PassThrough<double>>(
      kConveyorBeltNumPositions + kConveyorBeltNumVelocities);

  builder.ExportInput(state_pass_through->get_input_port(), "state");

  auto command_receiver = builder.AddSystem<ConveyorBeltCommandReceiver>();
  builder.ExportInput(command_receiver->GetInputPort("command_message"),
                      "command_message");

  auto wsg_trajectory_generator =
      builder.AddSystem<ConveyorBeltTrajectoryGenerator>(
          kConveyorBeltNumPositions + kConveyorBeltNumVelocities,
          kConveyorBeltPositionIndex);
  builder.Connect(state_pass_through->get_output_port(),
                  wsg_trajectory_generator->get_state_input_port());
  builder.Connect(command_receiver->get_position_output_port(),
                  wsg_trajectory_generator->get_desired_position_input_port());
  builder.Connect(command_receiver->get_force_limit_output_port(),
                  wsg_trajectory_generator->get_force_limit_input_port());

  auto wsg_controller = builder.AddSystem<ConveyorBeltPlainController>(
      ControlMode::kPosition, kp, ki, kd);
  builder.Connect(state_pass_through->get_output_port(),
                  wsg_controller->get_input_port_estimated_state());
  builder.Connect(wsg_trajectory_generator->get_target_output_port(),
                  wsg_controller->get_input_port_desired_state());
  builder.Connect(wsg_trajectory_generator->get_max_force_output_port(),
                  wsg_controller->get_input_port_max_force());

  builder.ExportOutput(wsg_controller->get_output_port_control(), "force");
  builder.BuildInto(this);
}

}  // namespace yaskawa_conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
