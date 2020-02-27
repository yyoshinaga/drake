#include <limits>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
// #include "drake/examples/yaskawa_arm/yaskawa_lcm.h"

#include "drake/geometry/geometry_visualization.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
// #include "drake/lcmt_schunk_wsg_command.hpp"
// #include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmt_yaskawa_ee_command.hpp"
#include "drake/lcmt_yaskawa_ee_status.hpp"


#include "drake/manipulation/yaskawa/yaskawa_command_receiver.h"	
#include "drake/manipulation/yaskawa/yaskawa_status_sender.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_lcm.h"

#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/matrix_gain.h"

//Add on
// #include "drake/examples/multibody/conveyor_belt/conveyor_station.h"


namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

// Runs a simulation of the manipulation station plant as a stand-alone
// simulation which mocks the network inputs and outputs of the real robot
// station.  This is a useful test in the transition from a single-process
// simulation to operating on the real robot hardware.

using Eigen::VectorXd;
// using examples::multibody::conveyor_belt::ConveyorStation;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, std::numeric_limits<double>::infinity(),
              "Simulation duration.");
DEFINE_string(setup, "manipulation_class",
              "Manipulation station type to simulate. "
              "Can be {manipulation_class, clutter_clearing}");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  // Create the "manipulation station".
  auto station = builder.AddSystem<ManipulationStation>();

  if (FLAGS_setup == "manipulation_class") {
    station->SetupManipulationClassStation();
    // station->AddManipulandFromFile(
    //     "drake/examples/manipulation_station/models/package1.sdf",
    //     math::RigidTransform<double>(math::RotationMatrix<double>::Identity(),
    //                                  Eigen::Vector3d(1.75, 1.999, 1.2)));
    // station->AddManipulandFromFile(
    //     "drake/examples/manipulation_station/models/package2.sdf",
    //     math::RigidTransform<double>(math::RollPitchYaw<double>(0, 0, 0.57),
    //                                  Eigen::Vector3d(1.75, 2.7, 1.05)));


  } else if (FLAGS_setup == "clutter_clearing") {
    station->SetupClutterClearingStation();
    station->AddManipulandFromFile(
        "drake/manipulation/models/ycb/sdf/003_cracker_box.sdf",
        math::RigidTransform<double>(math::RollPitchYaw<double>(-1.57, 0, 3),
                                     Eigen::Vector3d(-0.3, -0.55, 0.36)));
  } else {
    throw std::domain_error(
        "Unrecognized station type. Options are "
        "{manipulation_class, clutter_clearing}.");
  }
  // TODO(russt): Load sdf objects specified at the command line.  Requires
  // #9747.
  drake::log()->info("going to finalize ");
  station->Finalize();

  geometry::ConnectDrakeVisualizer(&builder, station->get_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));

  // geometry::ConnectDrakeVisualizer(&builder, conveyor_station->get_scene_graph(), 
  //                                  conveyor_station->GetOutputPort("pose_bundle2"));

  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  auto iiwa_command_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm));

  auto iiwa_command =
      builder.AddSystem<manipulation::yaskawa::YaskawaCommandReceiver>();
  builder.Connect(iiwa_command_subscriber->get_output_port(),
                  iiwa_command->get_input_port());

  // Pull the positions out of the state.
  builder.Connect(iiwa_command->get_commanded_position_output_port(),
                  station->GetInputPort("iiwa_position"));
  builder.Connect(iiwa_command->get_commanded_torque_output_port(),
                  station->GetInputPort("iiwa_feedforward_torque"));

  auto iiwa_status =
      builder.AddSystem<manipulation::yaskawa::YaskawaStatusSender>();
  builder.Connect(station->GetOutputPort("iiwa_position_commanded"),
                  iiwa_status->get_position_commanded_input_port());
  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  iiwa_status->get_position_measured_input_port());
  builder.Connect(station->GetOutputPort("iiwa_velocity_estimated"),
                  iiwa_status->get_velocity_estimated_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_commanded"),
                  iiwa_status->get_torque_commanded_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_measured"),
                  iiwa_status->get_torque_measured_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_external"),
                  iiwa_status->get_torque_external_input_port());
  auto iiwa_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm, 0.005 /* publish period */));
  builder.Connect(iiwa_status->get_output_port(),
                  iiwa_status_publisher->get_input_port());

//   // Receive the CONVEYOR commands.
//   auto conveyor_command_subscriber = builder.AddSystem(
//       systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_conveyor_belt_command>(
//           "CONVEYOR_BELT_COMMAND", lcm));
//   auto conveyor_command =
//       builder.AddSystem<manipulation::conveyor_belt::ConveyorBeltCommandReceiver>();
//   builder.Connect(conveyor_command_subscriber->get_output_port(),
//                   conveyor_command->GetInputPort("command_message"));
//   builder.Connect(conveyor_command->get_velocity_output_port(),
//                   station->GetInputPort("conveyor_velocity"));

//   // Publish the CONVEYOR status.
//   auto conveyor_status =
//       builder.AddSystem<manipulation::conveyor_belt::ConveyorBeltStatusSender>();
//   builder.Connect(station->GetOutputPort("generalized_velocity"),
//                   conveyor_status->get_state_input_port());
//   builder.Connect(station->GetOutputPort("generalized_velocity"),
//                   conveyor_status->get_position_input_port());
//   auto conveyor_status_publisher = builder.AddSystem(
//       systems::lcm::LcmPublisherSystem::Make<drake::lcmt_conveyor_belt_status>(
//           "CONVEYOR_BELT_STATUS", lcm, 0.05 /* publish period */));
//   builder.Connect(conveyor_status->get_output_port(0),
//                   conveyor_status_publisher->get_input_port());




  // Receive the ee commands.
  auto ee_command_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_yaskawa_ee_command>(
          "EE_COMMAND", lcm));
  auto ee_command =
      builder.AddSystem<manipulation::yaskawa_conveyor_belt_dof1::EndEffectorCommandReceiver>();
  builder.Connect(ee_command_subscriber->get_output_port(),
                  ee_command->GetInputPort("command_message"));



DRAKE_DEMAND(false);
//FIX ME: 
/*
    Problem: Need to find out how Dorabot controls their ee motors. 
    PWM? PPM? Is the input acceleration or voltage? How accurate do we need to model the motor? 
    Depending on that, we need to change the command actuation port. 
*/

  //***Create acceleration_output_port instead of position_output_port***
  builder.Connect(ee_command->get_actuation_output_port(),
                  station->GetInputPort("ee_actuation"));
//   builder.Connect(ee_command->get_position_output_port(),
//                   station->GetInputPort("ee_position"));

  // Publish the ee status.
  auto ee_status =
      builder.AddSystem<manipulation::yaskawa_conveyor_belt_dof1::EndEffectorStatusSender>();
  builder.Connect(station->GetOutputPort("ee_state_measured"),
                  ee_status->get_state_input_port());

  auto ee_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_yaskawa_ee_status>(
          "EE_STATUS", lcm, 0.05 /* publish period */));
  builder.Connect(ee_status->get_output_port(0),
                  ee_status_publisher->get_input_port());
  drake::log()->info("finished connecting ee stuff in mock station");

  // TODO(russt): Publish the camera outputs.

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();
  auto& station_context =
      diagram->GetMutableSubsystemContext(*station, &context);


  // Get the initial Iiwa pose and initialize the iiwa_command to match.
  VectorXd q0 = station->GetIiwaPosition(station_context); //Yaskawa has q0 = size 6

  iiwa_command->set_initial_position(
      &diagram->GetMutableSubsystemContext(*iiwa_command, &context), q0);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.AdvanceTo(FLAGS_duration);                              
  drake::log()->info("finished simulation");

  return 0;
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::manipulation_station::do_main(argc, argv);
}
