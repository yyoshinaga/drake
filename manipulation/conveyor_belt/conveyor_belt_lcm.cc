#include "drake/manipulation/conveyor_belt/conveyor_belt_lcm.h"

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_conveyor_belt_command.hpp"
#include "drake/lcmt_conveyor_belt_status.hpp"

namespace drake {
namespace manipulation {
namespace conveyor_belt {

using systems::BasicVector;
using systems::Context;


// For simulation
ConveyorBeltCommandReceiver::ConveyorBeltCommandReceiver(double initial_velocity)
    :  initial_velocity_(initial_velocity) {
  this->DeclareVectorOutputPort("velocity", BasicVector<double>(1),
                                &ConveyorBeltCommandReceiver::CalcVelocityOutput);

  lcmt_conveyor_belt_command uninitialized_message{};
  this->DeclareAbstractInputPort(
      "command_message",
      Value<lcmt_conveyor_belt_command>(uninitialized_message));
}

void ConveyorBeltCommandReceiver::CalcVelocityOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message =
      this->get_input_port(0).Eval<lcmt_conveyor_belt_command>(context);

  double target_velocity = initial_velocity_; //Should only specify velocity of arm_base link
  if (message.utime != 0.0) {
    target_velocity = message.target_velocity / 1e3;
    if (std::isnan(target_velocity)) {
      target_velocity = 0;
    }
  }
  // output->SetAtIndex(0, target_velocity); //Fix to have send velocity of multiple links

}

// For Controller
ConveyorBeltCommandSender::ConveyorBeltCommandSender()
      : velocity_input_port_(this->DeclareVectorInputPort(
                              "velocity", systems::BasicVector<double>(1))
                               .get_index()) {
  this->DeclareAbstractOutputPort("lcmt_conveyor_belt_command",
                                  &ConveyorBeltCommandSender::CalcCommandOutput);
}

void ConveyorBeltCommandSender::CalcCommandOutput(
    const drake::systems::Context<double>& context,
    drake::lcmt_conveyor_belt_command* output) const {
  lcmt_conveyor_belt_command& command = *output;

  command.utime = context.get_time() * 1e6;
  command.target_velocity = get_velocity_input_port().Eval(context) * 1e3;
  
}


// For controller
ConveyorBeltStatusReceiver::ConveyorBeltStatusReceiver()
    : state_output_port_(this->DeclareVectorOutputPort(
                                 "state", systems::BasicVector<double>(2),
                                 &ConveyorBeltStatusReceiver::CopyStateOut)
                             .get_index()),
      position_output_port_(this->DeclareVectorOutputPort(
                                 "position", systems::BasicVector<double>(1),
                                 &ConveyorBeltStatusReceiver::CopyPositionOut)
                             .get_index()) {
  this->DeclareAbstractInputPort("lcmt_conveyor_belt_status",
                                 Value<lcmt_conveyor_belt_status>());
}

void ConveyorBeltStatusReceiver::CopyStateOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& status =
      get_status_input_port().Eval<lcmt_conveyor_belt_status>(context);

  output->SetAtIndex(0, status.actual_position / 1e3);
  
  // output->SetAtIndex(1, status.actual_speed_m_per_s / 1e3);
}

void ConveyorBeltStatusReceiver::CopyPositionOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& status =
      get_status_input_port().Eval<lcmt_conveyor_belt_status>(context);
  output->SetAtIndex(0, status.actual_position);
}

// For Simulation
ConveyorBeltStatusSender::ConveyorBeltStatusSender() {
  state_input_port_ =
      this->DeclareInputPort(systems::kVectorValued, 2).get_index();
  position_input_port_ =
      this->DeclareInputPort(systems::kVectorValued, 1).get_index();
  this->DeclareAbstractOutputPort(&ConveyorBeltStatusSender::OutputStatus);
}

void ConveyorBeltStatusSender::OutputStatus(const Context<double>& context,
                                         lcmt_conveyor_belt_status* output) const {
  lcmt_conveyor_belt_status& status = *output;
  status.utime = context.get_time() * 1e6;

  const auto& state = get_state_input_port().Eval(context);
  // The position and speed reported in this message are between the
  // two fingers rather than the position/speed of a single finger
  // (so effectively doubled).
    status.actual_position = state * 1e3;


  // status.actual_speed_m_per_s = state[1] * 1e3;

  // if (get_position_input_port().HasValue(context)) {
  //   status.actual_velocity = get_position_input_port().Eval(context)[0]);
  // } else {
  //   status.actual_velocity = 0;
  // }
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
