#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

/// This class implements a controller for a Schunk WSG gripper.  It
/// has two input ports: lcmt_conveyor_belt_dof1_command message and the current
/// state, and an output port which emits the target force for the actuated
/// finger. Note, only one of the command input ports should be connected,
/// However, if both are connected, the message input will be ignored.
/// The internal implementation consists of a PID controller (which controls
/// the target position from the command message) combined with a saturation
/// block (which applies the force control from the command message).
///
/// @system{
///   @input_port{state}
///   @input_port{command_message}
///   @output_port{force}
/// }
///
/// @ingroup manipulation_systems
class ConveyorBeltController : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConveyorBeltController)

  // The gains here are somewhat arbitrary.  The goal is to make sure
  // that the maximum force is generated except when very close to the
  // target.
  explicit ConveyorBeltController(double kp = 2000.0, double ki = 0.0,
                               double kd = 5.0);
};

}  // namespace yaskawa_conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
