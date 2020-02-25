#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_lcm.h"

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"


namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

using systems::BasicVector;
using systems::Context;

EndEffectorCommandReceiver::EndEffectorCommandReceiver(double initial_position,
                                                      double initial_force, 
                                                      double initial_pusher_position, 
                                                      double initial_pusher_velocity, 
                                                      double initial_puller_position, 
                                                      double initial_puller_velocity)
    : initial_position_(initial_position), initial_force_(initial_force),
    initial_pusher_position_(initial_pusher_position), initial_pusher_velocity_(initial_pusher_velocity),
    initial_puller_position_(initial_puller_position), initial_puller_velocity_(initial_puller_velocity) {

  this->DeclareVectorOutputPort("position", BasicVector<double>(3),
                                &EndEffectorCommandReceiver::CalcPositionOutput);
  this->DeclareVectorOutputPort(
      "force_limit", BasicVector<double>(1),
      &EndEffectorCommandReceiver::CalcForceLimitOutput);

  lcmt_yaskawa_ee_command uninitialized_message{};
  this->DeclareAbstractInputPort(
      "command_message",
      Value<lcmt_yaskawa_ee_command>(uninitialized_message));
}

void EndEffectorCommandReceiver::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message =
      this->get_input_port(0).Eval<lcmt_yaskawa_ee_command>(context);

  double target_position = initial_position_;
  if (message.utime != 0.0) {
    target_position = message.target_position_mm / 1e3;
    if (std::isnan(target_position)) {
      target_position = 0;
    }
  }

  output->SetAtIndex(0, target_position);
  output->SetAtIndex(1,initial_pusher_position_);  //Pusher
  output->SetAtIndex(2,initial_puller_position_);  //Puller
}

void EndEffectorCommandReceiver::CalcForceLimitOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message =
      this->get_input_port(0).Eval<lcmt_yaskawa_ee_command>(context);

  double force_limit = initial_force_;
  if (message.utime != 0.0) {
    force_limit = message.force;
  }

  drake::log()->info("pusher and puller velocity... please ignore this: {} {}", initial_pusher_velocity_, initial_puller_velocity_);
  drake::log()->info("conveyor_belt_dof1_lcm calcforcelimitoutput");
  output->SetAtIndex(0, force_limit);
}
//------------------------------------------------------------------------------------------------------------
EndEffectorCommandSender::EndEffectorCommandSender()
    : position_input_port_(this->DeclareVectorInputPort(
                                   "position", systems::BasicVector<double>(3))
                               .get_index()),
      force_limit_input_port_(
          this->DeclareVectorInputPort("force_limit",
                                       systems::BasicVector<double>(1))
              .get_index()) {
  this->DeclareAbstractOutputPort("lcmt_yaskawa_ee_command",
                                  &EndEffectorCommandSender::CalcCommandOutput);
}

void EndEffectorCommandSender::CalcCommandOutput(
    const drake::systems::Context<double>& context,
    drake::lcmt_yaskawa_ee_command* output) const {
  lcmt_yaskawa_ee_command& command = *output;

  command.utime = context.get_time() * 1e6;
  command.target_position_mm = get_position_input_port().Eval(context)[0] * 1e3;
  command.force = get_force_limit_input_port().Eval(context)[0];
}
//------------------------------------------------------------------------------------------------------------

EndEffectorStatusReceiver::EndEffectorStatusReceiver()
    : state_output_port_(this->DeclareVectorOutputPort(
                                 "state", systems::BasicVector<double>(6),
                                 &EndEffectorStatusReceiver::CopyStateOut)
                             .get_index()),
      force_output_port_(this->DeclareVectorOutputPort(
                                 "force", systems::BasicVector<double>(1),
                                 &EndEffectorStatusReceiver::CopyForceOut)
                             .get_index()) {
  this->DeclareAbstractInputPort("lcmt_yaskawa_ee_status",
                                 Value<lcmt_yaskawa_ee_status>());
}

void EndEffectorStatusReceiver::CopyStateOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& status =
      get_status_input_port().Eval<lcmt_yaskawa_ee_status>(context);
  output->SetAtIndex(0, status.actual_position_mm / 1e3);
  output->SetAtIndex(3, status.actual_speed_mm_per_s / 1e3);

  output->SetAtIndex(1,status.actual_pusher_position);
  output->SetAtIndex(2,status.actual_puller_position);
  output->SetAtIndex(4,status.actual_pusher_speed);
  output->SetAtIndex(5,status.actual_puller_speed);
}

void EndEffectorStatusReceiver::CopyForceOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& status =
      get_status_input_port().Eval<lcmt_yaskawa_ee_status>(context);
  output->SetAtIndex(0, status.actual_force);
  output->SetAtIndex(1, status.actual_pusher_speed);
  output->SetAtIndex(2, status.actual_puller_speed);

}
//------------------------------------------------------------------------------------------------------------

EndEffectorStatusSender::EndEffectorStatusSender() {
  state_input_port_ =
      this->DeclareInputPort(systems::kVectorValued, 6).get_index();
  force_input_port_ =
      this->DeclareInputPort(systems::kVectorValued, 1).get_index();
  //Do we need acceleration input port?
  // acc_input_port_ =
  //     this->DeclareInputPort(systems::kVectorValued, 2).get_index();

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
  status.actual_position_mm = state[0] * 1e3;
  status.actual_speed_mm_per_s = state[3] * 1e3;

  if (get_force_input_port().HasValue(context)) {
    // In drake-schunk-driver, the driver attempts to apply a sign to the
    // force value based on the last reported direction of gripper movement
    // (the gripper always reports positive values).  This does not work very
    // well, and as a result the sign is almost always negative (when
    // non-zero) regardless of which direction the gripper was moving in when
    // motion was blocked.  As it's not a reliable source of information in
    // the driver (and should be removed there at some point), we don't try to
    // replicate it here and instead report positive forces.
    using std::abs;
    status.actual_force = abs(get_force_input_port().Eval(context)[0]);
  } else {
    status.actual_force = 0;
  }

  status.actual_pusher_position = state[1];
  status.actual_puller_position = state[2];
  status.actual_pusher_speed = state[4];
  status.actual_puller_speed = state[5];
}

}  // namespace conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
