#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_lcm.h"
#include <gflags/gflags.h>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"


namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

using systems::BasicVector;
using systems::Context;

EndEffectorCommandReceiver::EndEffectorCommandReceiver() {

  this->DeclareVectorOutputPort("actuation", BasicVector<double>(2),
                                &EndEffectorCommandReceiver::CalcActuationOutput);

  lcmt_yaskawa_ee_command uninitialized_message{};
  this->DeclareAbstractInputPort(
      "command_message",
      Value<lcmt_yaskawa_ee_command>(uninitialized_message));
}

//TODO: Find out how Dorabot controls their ee motors
void EndEffectorCommandReceiver::CalcActuationOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message =
      this->get_input_port(0).Eval<lcmt_yaskawa_ee_command>(context);

  output->SetAtIndex(0, message.whiskers_down?1:0); 
  output->SetAtIndex(1, message.roller_movement); 
}

//------------------------------------------------------------------------------------------------------------
EndEffectorCommandSender::EndEffectorCommandSender()
    : position_input_port_(this->DeclareVectorInputPort(
                                   "position", systems::BasicVector<double>(2))
                               .get_index()) {
  this->DeclareAbstractOutputPort("lcmt_yaskawa_ee_command",
                                  &EndEffectorCommandSender::CalcCommandOutput);
}

void EndEffectorCommandSender::CalcCommandOutput(
    const drake::systems::Context<double>& context,
    drake::lcmt_yaskawa_ee_command* output) const {
  lcmt_yaskawa_ee_command& command = *output;

  command.utime = context.get_time() * 1e6;
  command.whiskers_down = get_position_input_port().Eval(context)[0];
  command.roller_movement = get_position_input_port().Eval(context)[1];

}
//------------------------------------------------------------------------------------------------------------

EndEffectorStatusReceiver::EndEffectorStatusReceiver()
    : state_output_port_(this->DeclareVectorOutputPort(
                                 "state", systems::BasicVector<double>(4),
                                 &EndEffectorStatusReceiver::CopyStateOut)
                             .get_index()) {
  this->DeclareAbstractInputPort("lcmt_yaskawa_ee_status",
                                 Value<lcmt_yaskawa_ee_status>());
}

void EndEffectorStatusReceiver::CopyStateOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& status =
      get_status_input_port().Eval<lcmt_yaskawa_ee_status>(context);
  output->SetAtIndex(0, status.actual_whisker_angle);
  output->SetAtIndex(1, status.actual_roller_position);

  output->SetAtIndex(2, status.actual_whisker_speed);
  output->SetAtIndex(3, status.actual_roller_speed);
}

//------------------------------------------------------------------------------------------------------------

EndEffectorStatusSender::EndEffectorStatusSender() {
  state_input_port_ =
      this->DeclareInputPort(systems::kVectorValued, 4).get_index();

  this->DeclareAbstractOutputPort(&EndEffectorStatusSender::OutputStatus);
}

void EndEffectorStatusSender::OutputStatus(const Context<double>& context,
                                         lcmt_yaskawa_ee_status* output) const {
  lcmt_yaskawa_ee_status& status = *output;
  status.utime = context.get_time() * 1e6;

  const auto& state = get_state_input_port().Eval(context);
  // The position and speed reported in this message are between the
  // two fingers rather than the position/speed of a single finger
  // (so effectively doubled).

  status.actual_whisker_angle = state[0];
  status.actual_roller_position = state[1];

  status.actual_whisker_speed = state[2];
  status.actual_roller_speed = state[3];
}

}  // namespace conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
