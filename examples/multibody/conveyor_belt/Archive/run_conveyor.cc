#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
// #include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/demultiplexer.h"

#include "drake/systems/primitives/wrap_to_system.h"
#include "drake/systems/primitives/port_switch.h"

namespace drake {

using geometry::SceneGraph;
using lcm::DrakeLcm;
// using multibody::benchmarks::acrobot::AcrobotParameters;
// using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::JointActuator;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using systems::Context;
using systems::InputPort;
using Eigen::Vector2d;

namespace examples {
namespace multibody {
namespace conveyor_belt {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "Desired duration of the simulation in seconds.");

DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates. "
    "If 'false', the plant is modeled as a continuous system.");


int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_time_stepping ? 1.0e-3 : 0.0;

  // Make and add the acrobot model.
  const std::string relative_name =
      "drake/manipulation/models/conveyor_belt_description/sdf/conveyor_simple_robot.sdf";
  const std::string full_name = FindResourceOrThrow(relative_name);
  MultibodyPlant<double>& belt =
      *builder.AddSystem<MultibodyPlant>(time_step);

  Parser parser(&belt, &scene_graph);
  auto instance = parser.AddModelFromFile(full_name);

  // We are done defining the model.
  belt.Finalize();

    //In order to make 1 actuator -> check sdf for "effort"
  DRAKE_DEMAND(belt.num_actuators() == 2);      
  DRAKE_DEMAND(belt.num_actuated_dofs() == 2);

  PrismaticJoint<double>& shoulder =
      belt.GetMutableJointByName<PrismaticJoint>("zeroth_joint");
  RevoluteJoint<double>& elbow =
      belt.GetMutableJointByName<RevoluteJoint>("first_joint");

  // Drake's parser will default the name of the actuator to match the name of
  // the joint it actuates.
  const JointActuator<double>& actuator =
      belt.GetJointActuatorByName("zeroth_joint");
  const JointActuator<double>& actuator2 =
      belt.GetJointActuatorByName("first_joint");
  DRAKE_DEMAND(actuator.joint().name() == "zeroth_joint");
  DRAKE_DEMAND(actuator2.joint().name() == "first_joint");

  // Size of controller is chosen by changing the gain sizes (Currently 2x1)
  auto controller = builder.AddSystem<systems::controllers::PidController>(
      Eigen::Matrix<double,2,1>::Constant(1.0), 
      Eigen::Matrix<double,2,1>::Constant(0.0), 
      Eigen::Matrix<double,2,1>::Constant(1.0)
  );

  // Constant source of 2x1 values - probably for 2 states
//   auto constant_source = builder.AddSystem<systems::ConstantVectorSource<double>>(
//       Eigen::Matrix<double,2,1>::Constant(-0.3));

  auto wrapped_source = builder.AddSystem<systems::WrapToSystem<double>>(4);
  wrapped_source->set_interval(0, 0.1, 1 );
  wrapped_source->set_interval(3, 0.6, 1.4 );

//   auto switch_source = builder.AddSystem<systems::PortSwitch<double>>(4);
//   const InputPort<double>& port_pris = switch_source->DeclareInputPort("Prismatic");
//   const InputPort<double>& port_rev = switch_source->DeclareInputPort("Revolute");



// //   // Takes the integral of the constant source of 2x1 values - probably for the other 2 states
//   auto integrator = builder.AddSystem<systems::Integrator<double>>(2);

//   // Creates a Multiplexer which just combines the matrices
//   std::vector<int> mux_sizes(2,2);
//   std::vector<int> demux_sizes(2,2);
//   std::vector<int> super_mux_sizes(2,2);

// //   auto mux = builder.AddSystem<systems::Multiplexer>(mux_sizes);
//   auto mux4 = builder.AddSystem<systems::Multiplexer>(super_mux_sizes);
//   auto demux = builder.AddSystem<systems::Demultiplexer>(demux_sizes);

  controller->set_name("controller");
  builder.Connect(belt.get_state_output_port(instance),
                  controller->get_input_port_estimated_state());

  builder.Connect(belt.get_state_output_port(instance),
                  wrapped_source->get_input_port(0));

  builder.Connect(wrapped_source->get_output_port(0),
                  controller->get_input_port_desired_state());

  builder.Connect(controller->get_output_port_control(),
                  belt.get_actuation_input_port());

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!belt.get_source_id());

  builder.Connect(
      belt.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(belt.get_source_id().value()));

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);

  RandomGenerator generator;

  // Setup distribution for random initial conditions.
  std::normal_distribution<symbolic::Expression> gaussian;
  shoulder.set_random_translation_distribution(0.02*gaussian(generator));
  elbow.set_random_angle_distribution(0.05*gaussian(generator));


  // diagram.SetDefaultState(*context, &context->get_mutable_state());

  for (int i = 0; i < 5; i++) {
    simulator.get_mutable_context().SetTime(0.0);
    simulator.get_system().SetRandomContext(&simulator.get_mutable_context(),
                                            &generator);

    simulator.Initialize();
    simulator.AdvanceTo(FLAGS_simulation_time);
  }

  return 0;
}

}  // namespace
}  // namespace conveyor
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyPlant with "
      "LQR stabilization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::conveyor_belt::do_main();
}
