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

#include "drake/examples/multibody/conveyor_belt/conveyor_station.h"
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

template <typename T>
ConveyorStation<T>::ConveyorStation(double time_step)
    : owned_plant_(std::make_unique<drake::multibody::MultibodyPlant<T>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()),
      owned_controller_plant_(std::make_unique<drake::multibody::MultibodyPlant<T>>(0.0)) {


  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  source_id_ = plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  scene_graph_->set_name("scene_graph2");
  plant_->set_name("plant2");

  this->set_name("conveyor_station");
}

template <typename T>
void ConveyorStation<T>::BuildEverything(){

  systems::DiagramBuilder<double> builder;

  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  // auto plant = builder.AddSystem(std::make_unique<MultibodyPlant<double>>());
  // auto scene_graph = builder.AddSystem(std::make_unique<SceneGraph<double>>());
  Parser parser(plant_, scene_graph_);

  const std::string path = FindResourceOrThrow(
          "drake/manipulation/models/conveyor_belt_description/sdf/conveyor_simple_robot.sdf");

  // source_id = plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  drake::multibody::ModelInstanceIndex modelInstanceName = parser.AddModelFromFile(path);

  //For some reason, frame_ids size is 4. Is it because of the robot configuration?
  std::vector<geometry::FrameId> frame_ids;
  for(int i = 0; i < 3; i++) {
    frame_ids.push_back({plant_->GetBodyFrameIdOrThrow(plant_->GetBodyIndices(modelInstanceName)[i])});
  }

  plant_->RegisterVisualGeometry(plant_->world_body(), math::RigidTransformd(), geometry::HalfSpace(), "Floor");
  plant_->Finalize();

  auto vector_source_system = builder.AddSystem(std::make_unique<ConveyorControl<double>>());
  auto robot_plant = builder.AddSystem(std::make_unique<ConveyorPlant<double>>(frame_ids)); 

  geometry::ConnectDrakeVisualizer(&builder, *scene_graph_);

  builder.Connect(vector_source_system->get_output_port(0), 
                  robot_plant->get_input_port(0));

  builder.Connect(robot_plant->get_output_port(1),
                  plant_->get_actuation_input_port());

  builder.Connect(robot_plant->get_output_port(2),
                  scene_graph_->get_source_pose_port(source_id_));
                  
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());


  builder.ExportOutput(scene_graph_->get_pose_bundle_output_port(),
                       "pose_bundle2");

  builder.BuildInto(this);
  
}


}  // namespace conveyor
}  // namespace multibody
}  // namespace examples
}  // namespace drake
template class ::drake::examples::multibody::conveyor_belt::ConveyorStation<double>;


