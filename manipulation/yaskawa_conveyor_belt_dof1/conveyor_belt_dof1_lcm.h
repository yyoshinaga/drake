#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the Schunk WSG gripper.

#include <memory>
#include <vector>

#include "drake/lcmt_yaskawa_ee_command.hpp"
#include "drake/lcmt_yaskawa_ee_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

/// Handles the command for the Schunk WSG gripper from a LcmSubscriberSystem.
///
/// It has one input port: "command_message" for lcmt_conveyor_belt_dof1_command
/// abstract values.
///
/// It has two output ports: one for the commanded finger position represented
/// as the desired distance between the fingers in meters, and one for the
/// commanded force limit.  The commanded position and force limit are scalars
/// (BasicVector<double> of size 1).
///
/// @system{ EndEffectorCommandReceiver,
///   @input_port{command_message},
///   @output_port{position}
///   @output_port{force_limit} }
class EndEffectorCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EndEffectorCommandReceiver)

  /// @param initial_position the commanded position to output if no LCM
  /// message has been received yet.

  EndEffectorCommandReceiver();

  const systems::OutputPort<double>& get_actuation_output_port() const {
    return this->GetOutputPort("actuation");
  }

 private:
  void CalcActuationOutput(const systems::Context<double>& context,
                          systems::BasicVector<double>* output) const;
};


/// Send lcmt_conveyor_belt_dof1_command messages for a Schunk WSG gripper.  Has
/// two input ports: one for the commanded finger position represented as the
/// desired signed distance between the fingers in meters, and one for the
/// commanded force limit.  The commanded position and force limit are
/// scalars (BasicVector<double> of size 1).
///
/// @system{ EndEffectorCommandSender,
///   @input_port{position}
///   @input_port{force_limit},
///   @output_port{lcmt_conveyor_belt_dof1_command}
/// }
class EndEffectorCommandSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EndEffectorCommandSender)

  EndEffectorCommandSender();

  const systems::InputPort<double>& get_command_input_port()
  const {
    return this->get_input_port(command_input_port_);
  }

  const systems::OutputPort<double>& get_command_output_port() const {
    return this->get_output_port(0);
  }

 private:
  void CalcCommandOutput(
      const systems::Context<double>& context,
      lcmt_yaskawa_ee_command* output) const;

 private:
  const systems::InputPortIndex command_input_port_{};
  // const systems::InputPortIndex pusher_position_input_port_{};
  // const systems::InputPortIndex puller_position_input_port_{};

};


/// Handles lcmt_conveyor_belt_dof1_status messages from a LcmSubscriberSystem.  Has
/// two output ports: one for the measured state of the gripper, represented as
/// the signed distance between the fingers in meters and its corresponding
/// velocity, and one for the measured force.
///
/// @system{ EndEffectorStatusReceiver,
///   @input_port{lcmt_conveyor_belt_dof1_status},
///   @output_port{state}
///   @output_port{force}
/// }
class EndEffectorStatusReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EndEffectorStatusReceiver)

  EndEffectorStatusReceiver();

  const systems::InputPort<double>& get_status_input_port() const {
    return this->get_input_port(0);
  }

  //Measuring pos b/w finger and vel
  const systems::OutputPort<double>& get_state_output_port()
  const {
    return this->get_output_port(state_output_port_);
  }

 private:
  void CopyStateOut(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const;

 private:
  const systems::OutputPortIndex state_output_port_{};

};


/// Sends lcmt_conveyor_belt_dof1_status messages for a Schunk WSG.  This
/// system has one input port for the current state of the WSG, and one
/// optional input port for the measured gripping force.
///
/// @system{ SchunkStatusSender,
///          @input_port{state}
///          @input_port{force},
///          @output_port{lcmt_conveyor_belt_dof1_status}
/// }
///
/// The state input is a BasicVector<double> of size 2 -- with one position
/// and one velocity -- representing the distance between the fingers (positive
/// implies non-penetration).
///
/// @ingroup manipulation_systems
class EndEffectorStatusSender : public systems::LeafSystem<double> {
 public:
  EndEffectorStatusSender();

  const systems::InputPort<double>& get_state_input_port() const {
    DRAKE_DEMAND(state_input_port_.is_valid());
    return this->get_input_port(state_input_port_);
  }

 private:
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_yaskawa_ee_status* output) const;

  systems::InputPortIndex state_input_port_{};
};

}  // namespace conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
