#include "drake/examples/multibody/conveyor_belt/conveyor_geometry.h"

#include <memory>

#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using Eigen::Matrix3d;

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

const ConveyorGeometry* ConveyorGeometry::AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& conveyor_state_port,
      geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto conveyor_geometry = builder->AddSystem(
      std::unique_ptr<ConveyorGeometry>(
          new ConveyorGeometry(scene_graph)));
  builder->Connect(
      conveyor_state_port,
      conveyor_geometry->get_input_port(0));
  builder->Connect(
      conveyor_geometry->get_output_port(0),
      scene_graph->get_source_pose_port(conveyor_geometry->source_id_));

  return conveyor_geometry;
}

ConveyorGeometry::ConveyorGeometry(
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  // Use (temporary) MultibodyPlant to parse the urdf and setup the
  // scene_graph.
  // TODO(SeanCurtis-TRI): Update this on resolution of #10775.
  multibody::MultibodyPlant<double> mbp(0.0);
  multibody::Parser parser(&mbp, scene_graph);

  auto model_id = parser.AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/conveyor_belt_description/sdf/conveyor_simple_robot.sdf"));
  mbp.Finalize();

  source_id_ = *mbp.get_source_id();
  frame_id_ = mbp.GetBodyFrameIdOrThrow(mbp.GetBodyIndices(model_id)[0]);

  this->DeclareVectorInputPort("state", systems::BasicVector<double>(3));
  this->DeclareAbstractOutputPort(
      "geometry_pose", &ConveyorGeometry::OutputGeometryPose);
}

ConveyorGeometry::~ConveyorGeometry() = default;

void ConveyorGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(link_frame_id_.is_valid());

  // TODO(russt): Use AcrobotState here upon resolution of #12566.
  const auto& input =
      get_input_port(0).Eval<systems::BasicVector<double>>(context);
  const double theta1 = input[0], theta2 = input[1];

  const math::RigidTransformd link_pose(
      math::RotationMatrixd::MakeYRotation(theta1 + theta2),
      Vector3d(-l1_ * std::sin(theta1), 0, -l1_ * std::cos(theta1)));

  *poses = {{link_frame_id_, link_pose}};
}

}   //namespace conveyor
}  // namespace multibody
}  // namespace examples
}  // namespace drake
