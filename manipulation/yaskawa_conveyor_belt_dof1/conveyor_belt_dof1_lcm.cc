#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_lcm.h"
#include <gflags/gflags.h>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"


namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

DEFINE_double(whiskers_P, 8.0, "Proportional gain");
DEFINE_double(whiskers_D, 8.0, "Derivative gain");
DEFINE_double(pusher_P, 8.0, "Proportional gain");
DEFINE_double(pusher_D, 8.0, "Derivative gain");
DEFINE_double(puller_P, 8.0, "Proportional gain");
DEFINE_double(puller_D, 8.0, "Derivative gain");

DEFINE_double(pusher_init, 8.0, "Initial position");
DEFINE_double(pusher_max, 8.0, "Max position");
DEFINE_double(puller_init, 8.0, "Initial position");
DEFINE_double(puller_max, 8.0, "Max position");

using systems::BasicVector;
using systems::Context;

EndEffectorCommandReceiver::EndEffectorCommandReceiver() {

  this->DeclareVectorOutputPort("actuation", BasicVector<double>(3),
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

  // // Get the states from the plant
  // const systems::BasicVector<double>* input_vector =
  //     this->EvalVectorInput(context, 0);
  // auto inputVector = input_vector->CopyToVector();

  // double whiskerAcc = 0;
  // double pusherAcc = 0;
  // double pullerAcc = 0;

  // //TODO: The acceleration calculations need to be modified.

  // //If the message transfer started and position is not infinity, 
  // if (message.utime != 0.0) {
  //   //If true, make whiskers go down. Otherwise move them back up
  //   if(message.whiskers_down){
  //     whiskerAcc = FLAGS_whiskers_P*(M_PI/2-inputVector[0])+FLAGS_whiskers_D*(-inputVector[3]);
  //   }
  //   else{
  //     whiskerAcc = FLAGS_whiskers_P*(-inputVector[0])+FLAGS_whiskers_D*(-inputVector[3]);
  //   }

  //   //If true, move pusher to max position. Otherwise move them back to initial position
  //   if(message.pusher_move){
  //     pusherAcc = FLAGS_pusher_P*(FLAGS_pusher_max-inputVector[1])+FLAGS_pusher_D*(-inputVector[4]);
  //   }
  //   else{
  //     pusherAcc = FLAGS_pusher_P*(FLAGS_pusher_init-inputVector[1])+FLAGS_pusher_D*(-inputVector[4]);
  //   }

  //   //If true, move pusher to max position. Otherwise move them back to initial position
  //   if(message.puller_move){
  //     pullerAcc = FLAGS_puller_P*(FLAGS_puller_max-inputVector[2])+FLAGS_puller_D*(-inputVector[5]);
  //   }
  //   else{
  //     pullerAcc = FLAGS_puller_P*(FLAGS_puller_init-inputVector[2])+FLAGS_puller_D*(-inputVector[5]);
  //   }
  // }

  

  output->SetAtIndex(0, message.whiskers_down?1:0);  
  output->SetAtIndex(1, message.pusher_move?1:0);  //Pusher is moving?
  output->SetAtIndex(2, message.puller_move?1:0);  //Puller is moving?
}

//------------------------------------------------------------------------------------------------------------
EndEffectorCommandSender::EndEffectorCommandSender()
    : position_input_port_(this->DeclareVectorInputPort(
                                   "position", systems::BasicVector<double>(3))
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
  command.pusher_move = get_position_input_port().Eval(context)[1];
  command.puller_move = get_position_input_port().Eval(context)[2];

}
//------------------------------------------------------------------------------------------------------------

EndEffectorStatusReceiver::EndEffectorStatusReceiver()
    : state_output_port_(this->DeclareVectorOutputPort(
                                 "state", systems::BasicVector<double>(6),
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
  output->SetAtIndex(1,status.actual_pusher_position);
  output->SetAtIndex(2,status.actual_puller_position);

  output->SetAtIndex(3, status.actual_whisker_speed);
  output->SetAtIndex(4,status.actual_pusher_speed);
  output->SetAtIndex(5,status.actual_puller_speed);
}

//------------------------------------------------------------------------------------------------------------

EndEffectorStatusSender::EndEffectorStatusSender() {
  state_input_port_ =
      this->DeclareInputPort(systems::kVectorValued, 6).get_index();

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
  status.actual_pusher_position = state[1];
  status.actual_puller_position = state[2];

  status.actual_whisker_speed = state[3];
  status.actual_pusher_speed = state[4];
  status.actual_puller_speed = state[5];
}

}  // namespace conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
