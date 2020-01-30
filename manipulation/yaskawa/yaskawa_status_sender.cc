#include "drake/manipulation/yaskawa/yaskawa_status_sender.h"

namespace drake {
namespace manipulation {
namespace yaskawa {

YaskawaStatusSender::YaskawaStatusSender(int num_joints)
    : num_joints_(num_joints),
      zero_vector_(Eigen::VectorXd::Zero(num_joints)) {
  this->DeclareInputPort(
      "position_commanded", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "position_measured", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "velocity_estimated", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "torque_commanded", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "torque_measured", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "torque_external", systems::kVectorValued, num_joints_);
  this->DeclareAbstractOutputPort(
      "lcmt_yaskawa_status", &YaskawaStatusSender::CalcOutput);
}

using InPort = systems::InputPort<double>;
const InPort& YaskawaStatusSender::get_position_commanded_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const InPort& YaskawaStatusSender::get_position_measured_input_port() const {
  return LeafSystem<double>::get_input_port(1);
}
const InPort& YaskawaStatusSender::get_velocity_estimated_input_port() const {
  return LeafSystem<double>::get_input_port(2);
}
const InPort& YaskawaStatusSender::get_torque_commanded_input_port() const {
  return LeafSystem<double>::get_input_port(3);
}
const InPort& YaskawaStatusSender::get_torque_measured_input_port() const {
  return LeafSystem<double>::get_input_port(4);
}
const InPort& YaskawaStatusSender::get_torque_external_input_port() const {
  return LeafSystem<double>::get_input_port(5);
}
const systems::OutputPort<double>& YaskawaStatusSender::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

namespace {
// Returns the first one of port1.Eval or port2.Eval that has a value.
// If min_num_connected is zero and both ports are empty, return zeros.
// If less than min_num_connected of (port1,port2) are connected, throws.
// If more than max_num_connected of (port1,port2) are connected, throws.
// If port2_tail is provided, a suffix of port2's value is returned.
Eigen::Ref<const Eigen::VectorXd> EvalFirstConnected(
    const systems::Context<double>& context,
    int min_num_connected, int max_num_connected, const Eigen::VectorXd& zeros,
    const InPort& port1, const InPort& port2,
    int port2_tail = -1) {
  const int total_connected =
      (port1.HasValue(context) ? 1 : 0) +
      (port2.HasValue(context) ? 1 : 0);
  if (total_connected > max_num_connected) {
    throw std::logic_error(fmt::format(
        "Both {} and {} cannot both be connected at the same time.",
        port1.GetFullDescription(), port2.GetFullDescription()));
  }
  if (total_connected < min_num_connected) {
    throw std::logic_error(fmt::format(
        "At least {} of {} or {} must be connected.",
        min_num_connected, port1.GetFullDescription(),
        port2.GetFullDescription()));
  }
  if (port1.HasValue(context)) {
    return port1.Eval(context);
  }
  if (port2.HasValue(context)) {
    if (port2_tail < 0) {
      return port2.Eval(context);
    } else {
      return port2.Eval(context).tail(port2_tail);
    }
  }
  return zeros;
}

}  // namespace

void YaskawaStatusSender::CalcOutput(
    const systems::Context<double>& context, lcmt_iiwa_status* output) const {
  const auto& position_commanded =
      get_position_commanded_input_port().Eval(context);
  const auto& position_measured =
      get_position_measured_input_port().Eval(context);
  const auto& velocity_estimated =
      get_velocity_estimated_input_port().HasValue(context) ?
      get_velocity_estimated_input_port().Eval(context) :
      zero_vector_.head(num_joints_);
  const auto& torque_commanded =
      get_torque_commanded_input_port().Eval(context);
  const auto& torque_measured = EvalFirstConnected(
      context, 1, 2, zero_vector_,
      get_torque_measured_input_port(),
      get_torque_commanded_input_port());
  const auto& torque_external = EvalFirstConnected(
      context, 0, 2, zero_vector_,
      get_torque_external_input_port(),
      get_torque_external_input_port());

  lcmt_iiwa_status& status = *output;
  status.utime = context.get_time() * 1e6;
  status.num_joints = num_joints_;
  status.joint_position_measured.resize(num_joints_, 0);
  status.joint_velocity_estimated.resize(num_joints_, 0);
  status.joint_position_commanded.resize(num_joints_, 0);
  status.joint_position_ipo.resize(num_joints_, 0);
  status.joint_torque_measured.resize(num_joints_, 0);
  status.joint_torque_commanded.resize(num_joints_, 0);
  status.joint_torque_external.resize(num_joints_, 0);
  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position_measured[i] = position_measured[i];
    status.joint_velocity_estimated[i] = velocity_estimated[i];
    status.joint_position_commanded[i] = position_commanded[i];
    status.joint_torque_commanded[i] = torque_commanded[i];
    status.joint_torque_measured[i] = torque_measured[i];
    status.joint_torque_external[i] = torque_external[i];
  }
}

}  // namespace yaskawa
}  // namespace manipulation
}  // namespace drake
