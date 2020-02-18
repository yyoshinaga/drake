#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"


#include "drake/examples/multibody/conveyor_belt/conveyor_plant.h"
#include "drake/examples/multibody/conveyor_belt/conveyor_control.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/scene_graph_inspector.h"

namespace drake {

using geometry::SceneGraph;
using lcm::DrakeLcm;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
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

DEFINE_bool(time_stepping, false, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates. "
    "If 'false', the plant is modeled as a continuous system.");


class ConveyorSimulation : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConveyorSimulation)

  ConveyorSimulation();
  ~ConveyorSimulation();
};

ConveyorSimulation::ConveyorSimulation() {

  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem(std::make_unique<MultibodyPlant<double>>());
  auto scene_graph = builder.AddSystem(std::make_unique<SceneGraph<double>>());
  Parser parser(plant, scene_graph);

  const std::string path = FindResourceOrThrow(
          "drake/manipulation/models/conveyor_belt_description/sdf/conveyor_simple_robot.sdf");

  geometry::SourceId source_id = plant->RegisterAsSourceForSceneGraph(scene_graph);
  drake::multibody::ModelInstanceIndex modelInstanceName = parser.AddModelFromFile(path);

  //For some reason, frame_ids size is 4. Is it because of the robot configuration?
  std::vector<geometry::FrameId> frame_ids;
  for(int i = 0; i < 3; i++) {
    frame_ids.push_back({plant->GetBodyFrameIdOrThrow(plant->GetBodyIndices(modelInstanceName)[i])});
  }

  plant->RegisterVisualGeometry(plant->world_body(), math::RigidTransformd(), geometry::HalfSpace(), "Floor");
  plant->Finalize();

  auto vector_source_system = builder.AddSystem(std::make_unique<ConveyorControl<double>>());
  auto robot_plant = builder.AddSystem(std::make_unique<ConveyorPlant<double>>(frame_ids)); 

  geometry::ConnectDrakeVisualizer(&builder, *scene_graph);

  builder.Connect(vector_source_system->get_output_port(0), 
                  robot_plant->get_input_port(0));

  builder.Connect(robot_plant->get_output_port(1),
                  plant->get_actuation_input_port());

  builder.Connect(robot_plant->get_output_port(2),
                  scene_graph->get_source_pose_port(source_id));
                  
  builder.Connect(scene_graph->get_query_output_port(),
                  plant->get_geometry_query_input_port());

  builder.BuildInto(this);
  
}

ConveyorSimulation::~ConveyorSimulation() {}


int do_main() {
  //Create the simulation
  auto diagram = std::make_unique<ConveyorSimulation>();

  systems::Simulator<double> simulator(*diagram,diagram->CreateDefaultContext());
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);

  auto& context = simulator.get_mutable_context();

  drake::log()->info("Context size: {}"); //Size is 21? why?
  drake::log()->info( context.num_continuous_states());

  simulator.get_mutable_context().SetTime(0.0);
 
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

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
