#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the Schunk WSG gripper.

#include <memory>
#include <vector>

#include "drake/lcmt_conveyor_belt_command.hpp"
#include "drake/lcmt_conveyor_belt_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace conveyor_belt {

/// Handles the command for the Schunk WSG gripper from a LcmSubscriberSystem.
///
/// It has one input port: "command_message" for lcmt_schunk_wsg_command
/// abstract values.
///
/// It has two output ports: one for the commanded finger position represented
/// as the desired distance between the fingers in meters, and one for the
/// commanded force limit.  The commanded position and force limit are scalars
/// (BasicVector<double> of size 1).
///
/// @system{ ConveyorBeltCommandReceiver,
///   @input_port{command_message},
///   @output_port{position}
///   @output_port{force_limit} }
class ConveyorBeltCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConveyorBeltCommandReceiver)

  ///
  /// @param initial_velocity the commanded force limit to output if no LCM
  /// message has been received yet.
  ConveyorBeltCommandReceiver(double initial_velocity = 1);

  const systems::OutputPort<double>& get_velocity_output_port() const {
    return this->GetOutputPort("velocity");
  }

 private:
  void CalcVelocityOutput(const systems::Context<double>& context,
                          systems::BasicVector<double>* output) const;
                          
  const double initial_velocity_{};
};


/// Send lcmt_schunk_wsg_command messages for a Schunk WSG gripper.  Has
/// two input ports: one for the commanded finger position represented as the
/// desired signed distance between the fingers in meters, and one for the
/// commanded force limit.  The commanded position and force limit are
/// scalars (BasicVector<double> of size 1).
///
/// @system{ ConveyorBeltCommandSender,
///   @input_port{position}
///   @input_port{force_limit},
///   @output_port{lcmt_schunk_wsg_command}
/// }
class ConveyorBeltCommandSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConveyorBeltCommandSender)

  ConveyorBeltCommandSender();

  const systems::InputPort<double>& get_velocity_input_port()
  const {
    return this->get_input_port(velocity_input_port_);
  }

  const systems::OutputPort<double>& get_command_output_port() const {
    return this->get_output_port(0);
  }

 private:
  void CalcCommandOutput(
      const systems::Context<double>& context,
      lcmt_conveyor_belt_command* output) const;

 private:
  const systems::InputPortIndex velocity_input_port_{};
};


/// Handles lcmt_schunk_wsg_status messages from a LcmSubscriberSystem.  Has
/// two output ports: one for the measured state of the gripper, represented as
/// the signed distance between the fingers in meters and its corresponding
/// velocity, and one for the measured force.
///
/// @system{ ConveyorBeltStatusReceiver,
///   @input_port{lcmt_schunk_wsg_status},
///   @output_port{state}
///   @output_port{force}
/// }
class ConveyorBeltStatusReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConveyorBeltStatusReceiver)

  ConveyorBeltStatusReceiver();

  const systems::InputPort<double>& get_status_input_port() const {
    return this->get_input_port(0);
  }

  const systems::OutputPort<double>& get_state_output_port()
  const {
    return this->get_output_port(state_output_port_);
  }

  const systems::OutputPort<double>& get_position_output_port()
  const {
    return this->get_output_port(position_output_port_);
  }

 private:
  void CopyStateOut(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const;

  void CopyPositionOut(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const;

 private:
  const systems::OutputPortIndex state_output_port_{};
  const systems::OutputPortIndex position_output_port_{};
};


/// Sends lcmt_schunk_wsg_status messages for a Schunk WSG.  This
/// system has one input port for the current state of the WSG, and one
/// optional input port for the measured gripping force.
///
/// @system{ SchunkStatusSender,
///          @input_port{state}
///          @input_port{force},
///          @output_port{lcmt_schunk_wsg_status}
/// }
///
/// The state input is a BasicVector<double> of size 2 -- with one position
/// and one velocity -- representing the distance between the fingers (positive
/// implies non-penetration).
///
/// @ingroup manipulation_systems
class ConveyorBeltStatusSender : public systems::LeafSystem<double> {
 public:
  ConveyorBeltStatusSender();

  const systems::InputPort<double>& get_state_input_port() const {
    DRAKE_DEMAND(state_input_port_.is_valid());
    return this->get_input_port(state_input_port_);
  }

  const systems::InputPort<double>& get_position_input_port() const {
    return this->get_input_port(position_input_port_);
  }

 private:
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_conveyor_belt_status* output) const;

  systems::InputPortIndex state_input_port_{};
  systems::InputPortIndex position_input_port_{};
};

}  // namespace conveyor_belt
}  // namespace manipulation
}  // namespace drake
