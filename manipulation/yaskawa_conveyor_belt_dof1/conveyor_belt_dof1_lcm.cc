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
// const int kNumRollers = 7;

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

  output->SetAtIndex(0, message.whiskers_down); 
  output->SetAtIndex(1, message.roller_movement); 
}

//------------------------------------------------------------------------------------------------------------
EndEffectorCommandSender::EndEffectorCommandSender()
    : command_input_port_(this->DeclareVectorInputPort(
                                   "command", systems::BasicVector<double>(2))
                               .get_index()) {
  this->DeclareAbstractOutputPort("lcmt_yaskawa_ee_command",
                                  &EndEffectorCommandSender::CalcCommandOutput);
}

//This function isn't used??
void EndEffectorCommandSender::CalcCommandOutput(
    const drake::systems::Context<double>& context,
    drake::lcmt_yaskawa_ee_command* output) const {

  lcmt_yaskawa_ee_command& command = *output;
  drake::log()->info("Sending rollmer movement commads: {}",get_command_input_port().Eval(context)[1]);
  command.utime = context.get_time() * 1e6;
  command.whiskers_down = get_command_input_port().Eval(context)[0];
  command.roller_movement = get_command_input_port().Eval(context)[1];

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

//This function never gets called?
void EndEffectorStatusReceiver::CopyStateOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {

  const auto& status =
      get_status_input_port().Eval<lcmt_yaskawa_ee_status>(context);

  output->SetAtIndex(0, status.actual_whisker_angle);
  output->SetAtIndex(1, status.actual_roller_position);
  output->SetAtIndex(2, status.actual_whisker_speed);
  output->SetAtIndex(3, status.actual_roller_speed);

  // for(int i = 1; i <= kNumRollers; i++){
  //   output->SetAtIndex(i, status.actual_roller_position);
  // }

  // output->SetAtIndex(1+kNumRollers, status.actual_whisker_speed);
  // for(int i = kNumRollers+2; i <= 2*kNumRollers+2; i++){
  //   output->SetAtIndex(2+kNumRollers+i, status.actual_roller_speed);
  // }
}

//------------------------------------------------------------------------------------------------------------

EndEffectorStatusSender::EndEffectorStatusSender() {
  //Whisker angle and speed, roller position and speed (x 7)
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


        //Roller position = maybe whisker speed 
        //whisker speed = roller position
        //roller speed = ?? double roller position??? 

  drake::log()->info("Sending time: {}",status.utime);

  status.actual_whisker_angle = state[0];
  status.actual_whisker_speed = state[1];

  status.actual_roller_position = state[2]; //correct
  status.actual_roller_speed = state[3];


  // if(state[2] > 0){
  //   DRAKE_DEMAND(false);
  // }

  drake::log()->info("ssstate size: {}",state.size());
  drake::log()->info("state {}: {}",0,state[0]);
  drake::log()->info("state {}: {}",1,state[1]);
  drake::log()->info("state {}: {}",2,state[2]);
  drake::log()->info("state {}: {}",3,state[3]);
  drake::log()->info("state {}: {}",4,state[4]);  //Why is this one not off memory?
  drake::log()->info("state {}: {}",5,state[5]);  //Off memory


}

}  // namespace conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
