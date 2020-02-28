#include "drake/examples/manipulation_station/manipulation_station.h"

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"

#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/sensors/rgbd_sensor.h"

#include <drake/systems/controllers/pid_controlled_system.h>
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/wrap_to_system.h"

#include "drake/examples/multibody/conveyor_belt/conveyor_plant.h"
#include "drake/examples/multibody/conveyor_belt/conveyor_control.h"
#include "drake/examples/multibody/conveyor_belt/conveyor_controller.h"

#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_position_controller.h"
#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_constants.h"




namespace drake {
namespace examples {
namespace manipulation_station {

using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::SceneGraph;
using geometry::render::MakeRenderEngineVtk;
using geometry::render::RenderEngineVtkParams;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RotationMatrix;
using drake::multibody::Joint;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;

namespace internal {

// TODO(amcastro-tri): Refactor this into schunk_wsg directory, and cover it
// with a unit test.  Potentially tighten the tolerance in
// station_simulation_test.
// @param gripper_body_frame_name Name of a frame that's attached to the
// gripper's main body.
SpatialInertia<double> MakeCompositeGripperInertia(
    const std::string& wsg_sdf_path,
    const std::string& gripper_body_frame_name) {
  MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant);
  parser.AddModelFromFile(wsg_sdf_path);
  plant.Finalize();
  const auto& frame = plant.GetFrameByName(gripper_body_frame_name);
  const auto& gripper_body = plant.GetRigidBodyByName(frame.body().name());
  const auto& left_finger = plant.GetRigidBodyByName("whisker_left_link");
  const auto& right_finger = plant.GetRigidBodyByName("whisker_right_link");
  const auto& left_slider = plant.GetJointByName("whisker_left_joint");
  const auto& right_slider = plant.GetJointByName("whisker_right_joint");

  const SpatialInertia<double>& M_GGo_G = gripper_body.default_spatial_inertia();
  const SpatialInertia<double>& M_LLo_L = left_finger.default_spatial_inertia();
  const SpatialInertia<double>& M_RRo_R = right_finger.default_spatial_inertia();

  auto CalcFingerPoseInGripperFrame = [](const Joint<double>& slider) {
    // Pose of the joint's parent frame P (attached on gripper body G) in the
    // frame of the gripper G.
    const RigidTransform<double> X_GP(
        slider.frame_on_parent().GetFixedPoseInBodyFrame());
    // Pose of the joint's child frame C (attached on the slider's finger body)
    // in the frame of the slider's finger F.
    const RigidTransform<double> X_FC(
        slider.frame_on_child().GetFixedPoseInBodyFrame());
    // When the slider's translational dof is zero, then P coincides with C.
    // Therefore:
    const RigidTransform<double> X_GF = X_GP * X_FC.inverse();
    return X_GF;
  };

  // Pose of left finger L in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GL(CalcFingerPoseInGripperFrame(left_slider));
  // Pose of right finger R in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GR(CalcFingerPoseInGripperFrame(right_slider));
  // Helper to compute the spatial inertia of a finger F in about the gripper's
  // origin Go, expressed in G.
  auto CalcFingerSpatialInertiaInGripperFrame =
      [](const SpatialInertia<double>& M_FFo_F,
         const RigidTransform<double>& X_GF) {
        const auto M_FFo_G = M_FFo_F.ReExpress(X_GF.rotation());
        const auto p_FoGo_G = -X_GF.translation();
        const auto M_FGo_G = M_FFo_G.Shift(p_FoGo_G);
        return M_FGo_G;
      };

  // Shift and re-express in G frame the finger's spatial inertias.
  const auto M_LGo_G = CalcFingerSpatialInertiaInGripperFrame(M_LLo_L, X_GL);
  const auto M_RGo_G = CalcFingerSpatialInertiaInGripperFrame(M_RRo_R, X_GR);
  // With everything about the same point Go and expressed in the same frame G,
  // proceed to compose into composite body C:
  // TODO(amcastro-tri): Implement operator+() in SpatialInertia.

  SpatialInertia<double> M_CGo_G = M_GGo_G;
  M_CGo_G += M_LGo_G;
  M_CGo_G += M_RGo_G;
  return M_CGo_G;
}

// TODO(russt): Get these from SDF instead of having them hard-coded (#10022).
void get_camera_poses(std::map<std::string, RigidTransform<double>>* pose_map) {
  pose_map->emplace("0", RigidTransform<double>(
                             RollPitchYaw<double>(2.549607, 1.357609, 2.971679),
                             Vector3d(-0.228895, -0.452176, 0.486308)));

  pose_map->emplace("1",
                    RigidTransform<double>(
                        RollPitchYaw<double>(2.617427, -1.336404, -0.170522),
                        Vector3d(-0.201813, 0.469259, 0.417045)));

  pose_map->emplace("2",
                    RigidTransform<double>(
                        RollPitchYaw<double>(-2.608978, 0.022298, 1.538460),
                        Vector3d(0.786258, -0.048422, 1.043315)));
}

// Load a SDF model and weld it to the MultibodyPlant.
// @param model_path Full path to the sdf model file. i.e. with
// FindResourceOrThrow
// @param model_name Name of the added model instance.
// @param parent Frame P from the MultibodyPlant to which the new model is
// welded to.
// @param child_frame_name Defines frame C (the child frame), assumed to be
// present in the model being added.
// @param X_PC Transformation of frame C relative to frame P.
template <typename T>
drake::multibody::ModelInstanceIndex AddAndWeldModelFrom(
    const std::string& model_path, const std::string& model_name,
    const drake::multibody::Frame<T>& parent, const std::string& child_frame_name,
    const RigidTransform<double>& X_PC, MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(!plant->HasModelInstanceNamed(model_name));

  drake::multibody::Parser parser(plant);
  //This line gives isPhysicallyValid()
  const drake::multibody::ModelInstanceIndex new_model =
      parser.AddModelFromFile(model_path, model_name);

  const auto& child_frame = plant->GetFrameByName(child_frame_name, new_model);
  plant->WeldFrames(parent, child_frame, X_PC);
  drake::log()->info("AddAndWeldModelFrom 1");
  return new_model;
}

}  // namespace internal

template <typename T>
ManipulationStation<T>::ManipulationStation(double time_step)
    : owned_plant_(std::make_unique<MultibodyPlant<T>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()),
      // Given the controller does not compute accelerations, it is irrelevant
      // whether the plant is continuous or discrete. We arbitrarily make it
      // continuous.
      owned_controller_plant_(std::make_unique<MultibodyPlant<T>>(0.0)) {
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  source_id_ = plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  scene_graph_->set_name("scene_graph");
  plant_->set_name("plant");

  this->set_name("manipulation_station");
  drake::log()->info("ManipulationStation 2");
}

template <typename T>
void ManipulationStation<T>::AddManipulandFromFile(
    const std::string& model_file, const RigidTransform<double>& X_WObject) {
  drake::multibody::Parser parser(plant_);
  const auto model_index =
      parser.AddModelFromFile(FindResourceOrThrow(model_file));
  const auto indices = plant_->GetBodyIndices(model_index);
  // Only support single-body objects for now.
  // Note: this could be generalized fairly easily... would just want to
  // set default/random positions for the non-floating-base elements below.
  DRAKE_DEMAND(indices.size() == 1);
  object_indexes_.push_back(model_index); //Store this variable to use to move the objects in simulation
  object_ids_.push_back(indices[0]);
  object_poses_.push_back(X_WObject);
  drake::log()->info("AddManipulandFromFile 3");

}

template <typename T>
void ManipulationStation<T>::SetupClutterClearingStation(
    const std::optional<const math::RigidTransform<double>>& X_WCameraBody,
    IiwaCollisionModel collision_model) {
  DRAKE_DEMAND(setup_ == Setup::kNone);
  setup_ = Setup::kClutterClearing;

  // Add the bins.
  {
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/bin.sdf");

    RigidTransform<double> X_WC(RotationMatrix<double>::MakeZRotation(M_PI_2),
                                Vector3d(-0.145, -0.63, 0.235));
    internal::AddAndWeldModelFrom(sdf_path, "bin1", plant_->world_frame(),
                                  "bin_base", X_WC, plant_);

    X_WC = RigidTransform<double>(RotationMatrix<double>::MakeZRotation(M_PI),
                                  Vector3d(0.5, -0.1, 0.235));
    internal::AddAndWeldModelFrom(sdf_path, "bin2", plant_->world_frame(),
                                  "bin_base", X_WC, plant_);
  }

  // Add the camera.
  {
    // Typical D415 intrinsics for 848 x 480 resolution, note that rgb and
    // depth are slightly different. And we are not able to model that at the
    // moment.
    // RGB:
    // - w: 848, h: 480, fx: 616.285, fy: 615.778, ppx: 405.418, ppy: 232.864
    // DEPTH:
    // - w: 848, h: 480, fx: 645.138, fy: 645.138, ppx: 420.789, ppy: 239.13
    // For this camera, we are going to assume that fx = fy, and we can compute
    // fov_y by: fy = height / 2 / tan(fov_y / 2)
    const double kFocalY = 645.;
    const int kHeight = 480;
    const int kWidth = 848;
    const double fov_y = std::atan(kHeight / 2. / kFocalY) * 2;
    geometry::render::DepthCameraProperties camera_properties(
        kWidth, kHeight, fov_y, default_renderer_name_, 0.1, 2.0);

    RegisterRgbdSensor("0", plant_->world_frame(),
                       X_WCameraBody.value_or(math::RigidTransform<double>(
                           math::RollPitchYaw<double>(-0.3, 0.8, 1.5),
                           Eigen::Vector3d(0, -1.5, 1.5))),
                       camera_properties);
  }

  AddDefaultIiwa(collision_model);
  // AddDefaultWsg();
  drake::log()->info("SetupClutterClearingStation 4");

}

template <typename T>
void ManipulationStation<T>::SetupManipulationClassStation(
    IiwaCollisionModel collision_model) {
  DRAKE_DEMAND(setup_ == Setup::kNone);
  setup_ = Setup::kManipulationClass;

  // Add the table and 80/20 workcell frame.
  {
    const double dz_table_top_robot_base = 0.01;
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/"
        "amazon_table_floor_only.sdf");

    RigidTransform<double> X_WT(
        Vector3d(0, 0, -dz_table_top_robot_base));
    internal::AddAndWeldModelFrom(sdf_path, "table", plant_->world_frame(),
                                  "amazon_table", X_WT, plant_);
  }
  
  // Add upper conveyor belt 
  { 
    const std::string sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/conveyor_belt_description/sdf/"
      "conveyor_belt_surface_only_collision.sdf"
    );

    RigidTransform<double> X_BM(RotationMatrix<double>::MakeZRotation(M_PI/2),
                                Vector3d(1.7018, 2.32042, 0.8636)); 
                                //Y value is (0.82042m (2'-8.3" offset dist) + 1.5m) = 2.32042m
                                //(offset dist b/w robot center and beginning of conveyor + center of conveyor)
                                //X value is (7"+12"+4') = 1.7018m 
                                //(approx: dist b/w shelf corner and belt side + belt width/2 + shelf length/2)

    internal::AddAndWeldModelFrom(sdf_path, "conveyor_joint_upper", plant_->world_frame(),
                              "link", X_BM, plant_);
  }

  // Add lower conveyor belt 
  { 
    const std::string sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/conveyor_belt_description/sdf/"
      "conveyor_belt_surface_only_collision_2.sdf"
    );
    RigidTransform<double> X_BM(RotationMatrix<double>::MakeZRotation(-M_PI_2),
                                Vector3d(1.85, -0.5, 0));//0.5207)); 
                                //Y value is 0.82042m (2-8.3') - 1.5m = -0.67958m but 
                                //would not want the box to fall to the floor so round down to -0.5m
                                //(subtraction this time)
                                //X value is ()
                                //Z value is height of the belt top to the ground (0.5207) subtracted by dz_table_top_robot_base (0.0127)

    internal::AddAndWeldModelFrom(sdf_path, "conveyor_joint_lower", plant_->world_frame(),
                              "link2", X_BM, plant_);

  }

  // Add the sorting shelf Left.
  {
    const double dy_table_center_to_center_robot_base = -1.99189262; //Excel calculations
    const double shelf_angle = -0.0872665; //-5 degrees

    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/sorting_shelf.sdf");

    RigidTransform<double> X_WC(
            RotationMatrix<double>::MakeZRotation(shelf_angle+M_PI/2),
            Vector3d(0, dy_table_center_to_center_robot_base, 0));
    internal::AddAndWeldModelFrom(sdf_path, "sort_shelf_left", plant_->world_frame(),
                                  "columns", X_WC, plant_);
  }

  // Add the sorting shelf right.
  {
    const double dy_table_center_to_center_robot_base = 1.99189262; //Excel calculations
    const double shelf_angle = 0.0872665; //5 degrees

    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/sorting_shelf.sdf");

    RigidTransform<double> X_WC(
            RotationMatrix<double>::MakeZRotation(shelf_angle-M_PI/2),
            Vector3d(0, dy_table_center_to_center_robot_base, 0));
    internal::AddAndWeldModelFrom(sdf_path, "sort_shelf_right", plant_->world_frame(),
                                  "columns", X_WC, plant_);
  }

  // Add the sorting shelf top.
  {
    const double dx_table_center_to_center_robot_base = -2.05; //Excel calculations

    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/sorting_shelf.sdf");

    RigidTransform<double> X_WC(
            RotationMatrix<double>::MakeZRotation(0),
            Vector3d(dx_table_center_to_center_robot_base, 0, 0));
    internal::AddAndWeldModelFrom(sdf_path, "sort_shelf_top", plant_->world_frame(),
                                  "columns", X_WC, plant_);
  }

  // Add the default iiwa/wsg models.
  AddDefaultIiwa(collision_model);
  AddDefaultWsg();
  // Add default cameras.
  {
    std::map<std::string, RigidTransform<double>> camera_poses;
    internal::get_camera_poses(&camera_poses);
    // Typical D415 intrinsics for 848 x 480 resolution, note that rgb and
    // depth are slightly different. And we are not able to model that at the
    // moment.
    // RGB:
    // - w: 848, h: 480, fx: 616.285, fy: 615.778, ppx: 405.418, ppy: 232.864
    // DEPTH:
    // - w: 848, h: 480, fx: 645.138, fy: 645.138, ppx: 420.789, ppy: 239.13
    // For this camera, we are going to assume that fx = fy, and we can compute
    // fov_y by: fy = height / 2 / tan(fov_y / 2)
    const double kFocalY = 645.;
    const int kHeight = 480;
    const int kWidth = 848;
    const double fov_y = std::atan(kHeight / 2. / kFocalY) * 2;
    geometry::render::DepthCameraProperties camera_properties(
        kWidth, kHeight, fov_y, default_renderer_name_, 0.1, 2.0);
    for (const auto& camera_pair : camera_poses) {
      RegisterRgbdSensor(camera_pair.first, plant_->world_frame(),
                         camera_pair.second, camera_properties);
    }
  }
  drake::log()->info("SetupManipulationClassStation 5");
}

template <typename T>
void ManipulationStation<T>::SetupPlanarIiwaStation() {
  DRAKE_DEMAND(setup_ == Setup::kNone);
  setup_ = Setup::kPlanarIiwa;

  // Add the tables.
  {
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/kuka_iiwa_arm/models/table/"
        "extra_heavy_duty_table_surface_only_collision.sdf");

    const double table_height = 0.7645;
    internal::AddAndWeldModelFrom(
        sdf_path, "robot_table", plant_->world_frame(), "link",
        RigidTransform<double>(Vector3d(0, 0, -table_height)), plant_);
    internal::AddAndWeldModelFrom(
        sdf_path, "work_table", plant_->world_frame(), "link",
        RigidTransform<double>(Vector3d(0.75, 0, -table_height)), plant_);
  }

  // Add planar iiwa model.
  {
    std::string sdf_path = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "planar_iiwa14_spheres_dense_elbow_collision.urdf");
    const auto X_WI = RigidTransform<double>::Identity();
    auto iiwa_instance = internal::AddAndWeldModelFrom(
        sdf_path, "iiwa", plant_->world_frame(), "iiwa_link_0", X_WI, plant_);
    RegisterIiwaControllerModel(
        sdf_path, iiwa_instance, plant_->world_frame(),
        plant_->GetFrameByName("iiwa_link_0", iiwa_instance), X_WI);
  }

  // Add the default wsg model.
  // AddDefaultWsg();
  drake::log()->info("SetupPlanarIiwaStation 6");
}

template <typename T>
int ManipulationStation<T>::num_iiwa_joints() const {
  DRAKE_DEMAND(iiwa_model_.model_instance.is_valid());
  drake::log()->info("num_iiwa_joints() 7");
  return plant_->num_positions(iiwa_model_.model_instance);
}

template <typename T>
void ManipulationStation<T>::SetDefaultState(
    const systems::Context<T>& station_context,
    systems::State<T>* state) const {
  // Call the base class method, to initialize all systems in this diagram.
  systems::Diagram<T>::SetDefaultState(station_context, state);

  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  DRAKE_DEMAND(object_ids_.size() == object_poses_.size());

  for (uint64_t i = 0; i < object_ids_.size(); i++) {
    plant_->SetFreeBodyPose(plant_context, &plant_state,
                            plant_->get_body(object_ids_[i]), object_poses_[i]);
  }

  // Use SetIiwaPosition to make sure the controller state is initialized to
  // the IIWA state.

  SetIiwaPosition(station_context, state, GetIiwaPosition(station_context));
  SetIiwaVelocity(station_context, state, VectorX<T>::Zero(num_iiwa_joints()));
  
  // Vector3<T> eePosition;
  // eePosition << 0, 0, 1; //x_pos, theta, alpha

  // SetWsgPosition(station_context, state, eePosition);
  // SetWsgVelocity(station_context, state, Vector3<T>::Zero(0));
  Vector3<T> conveyorPosition1;
  conveyorPosition1 << 0.25, 0, 0; //x_pos, theta, alpha
  SetConveyorPosition(station_context, state, conveyorPosition1, 1);

  // Vector2<T> conveyorPosition2(2);
  // conveyorPosition2 << 0.16, 0; //x_pos, theta
  // SetConveyorPosition(station_context, state, conveyorPosition2, 2);

  // Vector2<T> conveyorPosition3(2);
  // conveyorPosition3 << 0.3, 0; //x_pos, theta
  // SetConveyorPosition(station_context, state, conveyorPosition3, 3);


  // SetConveyorVelocity(station_context, state, VectorX<T>::Zero(2));
  drake::log()->info("SetDefaultState 8");
}

template <typename T>
void ManipulationStation<T>::SetRandomState(
    const systems::Context<T>& station_context, systems::State<T>* state,
    RandomGenerator* generator) const {
  // Call the base class method, to initialize all systems in this diagram.
  systems::Diagram<T>::SetRandomState(station_context, state, generator);

  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  // Separate the objects by lifting them up in z (in a random order).
  // TODO(russt): Replace this with an explicit projection into a statically
  // stable configuration.
  std::vector<drake::multibody::BodyIndex> shuffled_object_ids(object_ids_);
  std::shuffle(shuffled_object_ids.begin(), shuffled_object_ids.end(),
               *generator);
  double z_offset = 0.1;
  for (const auto body_index : shuffled_object_ids) {
    math::RigidTransform<T> pose =
        plant_->GetFreeBodyPose(plant_context, plant_->get_body(body_index));
    pose.set_translation(pose.translation() + Vector3d{0, 0, z_offset});
    z_offset += 0.1;
    plant_->SetFreeBodyPose(plant_context, &plant_state,
                            plant_->get_body(body_index), pose);
  }

  // Use SetIiwaPosition to make sure the controller state is initialized to
  // the IIWA state.

  SetIiwaPosition(station_context, state, GetIiwaPosition(station_context));
  SetIiwaVelocity(station_context, state, VectorX<T>::Zero(num_iiwa_joints()));
  // SetWsgPosition(station_context, state, GetWsgPosition(station_context));
  // SetWsgVelocity(station_context, state, 0);   
  drake::log()->info("SetRandomState 9");

}

template <typename T>
void ManipulationStation<T>::MakeIiwaControllerModel() {
  // Build the controller's version of the plant, which only contains the
  // IIWA and the equivalent inertia of the gripper.
  drake::multibody::Parser parser(owned_controller_plant_.get());
  const auto controller_iiwa_model =
      parser.AddModelFromFile(iiwa_model_.model_path, "yaskawa");

  owned_controller_plant_->WeldFrames(
      owned_controller_plant_->world_frame(),
      owned_controller_plant_->GetFrameByName(iiwa_model_.child_frame->name(),
                                              controller_iiwa_model),
      iiwa_model_.X_PC);
  // Add a single body to represent the IIWA pendant's calibration of the
  // gripper.  The body of the WSG accounts for >90% of the total mass
  // (according to the sdf)... and we don't believe our inertia calibration
  // on the hardware to be so precise, so we simply ignore the inertia
  // contribution from the fingers here.
  const drake::multibody::RigidBody<T>& wsg_equivalent =
      owned_controller_plant_->AddRigidBody(
          "wsg_equivalent", controller_iiwa_model,
          internal::MakeCompositeGripperInertia(
              wsg_model_.model_path, wsg_model_.child_frame->name()));

  // TODO(siyuan.feng@tri.global): when we handle multiple IIWA and WSG, this
  // part need to deal with the parent's (iiwa's) model instance id.
  owned_controller_plant_->WeldFrames(
      owned_controller_plant_->GetFrameByName(wsg_model_.parent_frame->name(),
                                              controller_iiwa_model),
      wsg_equivalent.body_frame(), wsg_model_.X_PC);
  owned_controller_plant_->set_name("controller_plant");
  drake::log()->info("MakeIiwaControllerModel 10");
}


template <typename T>
void ManipulationStation<T>::Finalize() {
  Finalize({});
  drake::log()->info("Finalize 11");
}

template <typename T>
void ManipulationStation<T>::Finalize(
    std::map<std::string, std::unique_ptr<geometry::render::RenderEngine>>
        render_engines) {
  DRAKE_THROW_UNLESS(iiwa_model_.model_instance.is_valid());
  // DRAKE_THROW_UNLESS(wsg_model_.model_instance.is_valid());

  //Conveyor belt stuff needs to come before belt_plant_->finalize
  std::vector<geometry::FrameId> frame_ids1; 
  // std::vector<geometry::FrameId> frame_ids2; 
  // std::vector<geometry::FrameId> frame_ids3;
  AddDefaultConveyor(conveyor_belt_idx_1_, 1, frame_ids1, "conveyor_1");
  // AddDefaultConveyor(conveyor_belt_idx_2_, 2, frame_ids2, "conveyor_2");
  // AddDefaultConveyor(conveyor_belt_idx_3_, 3, frame_ids3, "conveyor_3");
  

  MakeIiwaControllerModel();

  // Note: This deferred diagram construction method/workflow exists because we
  //   - cannot finalize plant until all of my objects are added, and
  //   - cannot wire up my diagram until we have finalized the plant.
  plant_->Finalize();

  // Set plant properties that must occur after finalizing the plant.
  VectorX<T> q0_iiwa(num_iiwa_joints());

  switch (setup_) {
    case Setup::kNone:
    case Setup::kManipulationClass: {
      // Set the initial positions of the IIWA to a comfortable configuration
      // inside the workspace of the station.
      q0_iiwa << 0, 0, 0, 0, 0, 0;

      std::uniform_real_distribution<symbolic::Expression> x(0.4, 0.65),
          y(-0.35, 0.35), z(0, 0.05);
      const Vector3<symbolic::Expression> xyz{x(), y(), z()};
      for (const auto body_index : object_ids_) {
        const drake::multibody::Body<T>& body = plant_->get_body(body_index);
        plant_->SetFreeBodyRandomPositionDistribution(body, xyz);
        plant_->SetFreeBodyRandomRotationDistributionToUniform(body);
      }
      break;
    }
    case Setup::kClutterClearing: {
      // Set the initial positions of the IIWA to a configuration right above
      // the picking bin.
      q0_iiwa << -1.57, 0.1, 0, -1.2, 0, 1.6;

      std::uniform_real_distribution<symbolic::Expression> x(-.35, 0.05),
          y(-0.8, -.55), z(0.3, 0.35);
      const Vector3<symbolic::Expression> xyz{x(), y(), z()};
      for (const auto body_index : object_ids_) {
        const drake::multibody::Body<T>& body = plant_->get_body(body_index);
        plant_->SetFreeBodyRandomPositionDistribution(body, xyz);
        plant_->SetFreeBodyRandomRotationDistributionToUniform(body);
      }
      break;
    }
    case Setup::kPlanarIiwa: {
      // Set initial positions of the IIWA, but now with only joints 2, 4,
      // and 6.
      q0_iiwa << 0.1, -1.2, 1.6;

      std::uniform_real_distribution<symbolic::Expression> x(0.4, 0.8),
          y(0, 0), z(0, 0.05);
      const Vector3<symbolic::Expression> xyz{x(), y(), z()};
      for (const auto body_index : object_ids_) {
        const drake::multibody::Body<T>& body = plant_->get_body(body_index);
        plant_->SetFreeBodyRandomPositionDistribution(body, xyz);
      }
      break;
    }
  }

  // Set the iiwa default configuration.
  const auto iiwa_joint_indices =
      plant_->GetJointIndices(iiwa_model_.model_instance);
  int q0_index = 0;
  for (const auto joint_index : iiwa_joint_indices) {
    drake::multibody::RevoluteJoint<T>* joint =
        dynamic_cast<drake::multibody::RevoluteJoint<T>*>(
            &plant_->get_mutable_joint(joint_index));
    // Note: iiwa_joint_indices includes the WeldJoint at the base.  Only set
    // the RevoluteJoints.
    if (joint) {
      joint->set_default_angle(q0_iiwa[q0_index++]);
    }
  }

  systems::DiagramBuilder<T> builder;

  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());

  const int num_iiwa_positions =
      plant_->num_positions(iiwa_model_.model_instance);
  DRAKE_THROW_UNLESS(num_iiwa_positions ==
                     plant_->num_velocities(iiwa_model_.model_instance));
  // Export the commanded positions via a PassThrough.
  auto iiwa_position =
      builder.template AddSystem<systems::PassThrough>(num_iiwa_positions);
  builder.ExportInput(iiwa_position->get_input_port(), "iiwa_position");
  builder.ExportOutput(iiwa_position->get_output_port(),
                       "iiwa_position_commanded");

  // Export iiwa "state" outputs.
  {
    auto demux = builder.template AddSystem<systems::Demultiplexer>(
        2 * num_iiwa_positions, num_iiwa_positions);
    builder.Connect(plant_->get_state_output_port(iiwa_model_.model_instance),
                    demux->get_input_port(0));
    builder.ExportOutput(demux->get_output_port(0), "iiwa_position_measured");
    builder.ExportOutput(demux->get_output_port(1), "iiwa_velocity_estimated");

    builder.ExportOutput(
        plant_->get_state_output_port(iiwa_model_.model_instance),
        "iiwa_state_estimated");
  }

  // Add the IIWA controller "stack".
  {
    owned_controller_plant_->Finalize();

    auto check_gains = [](const VectorX<double>& gains, int size) {
      return (gains.size() == size) && (gains.array() >= 0).all();
    };

    // Set default gains if.
    if (iiwa_kp_.size() == 0) {
      iiwa_kp_ = VectorXd::Constant(num_iiwa_positions, 100);
    }
    DRAKE_THROW_UNLESS(check_gains(iiwa_kp_, num_iiwa_positions));

    if (iiwa_kd_.size() == 0) {
      iiwa_kd_.resize(num_iiwa_positions);
      for (int i = 0; i < num_iiwa_positions; i++) {
        // Critical damping gains.
        iiwa_kd_[i] = 2 * std::sqrt(iiwa_kp_[i]);
      }
    }
    DRAKE_THROW_UNLESS(check_gains(iiwa_kd_, num_iiwa_positions));

    if (iiwa_ki_.size() == 0) {
      iiwa_ki_ = VectorXd::Constant(num_iiwa_positions, 1);
    }
    DRAKE_THROW_UNLESS(check_gains(iiwa_ki_, num_iiwa_positions));

    // Add the inverse dynamics controller.
    auto iiwa_controller = builder.template AddSystem<
        systems::controllers::InverseDynamicsController>(
        *owned_controller_plant_, iiwa_kp_, iiwa_ki_, iiwa_kd_, false);
    iiwa_controller->set_name("iiwa_controller");

    // Demux the plant output ports (Essentially, it seperates it from the conveyor ports)
    // auto demux = builder.AddSystem<systems::Demultiplexer>(14, 2);
    // auto mux = builder.AddSystem<systems::Multiplexer>(6);
    
    // builder.Connect(plant_->get_state_output_port(iiwa_model_.model_instance),
    //                 demux->get_input_port(0));

    // builder.Connect(demux->get_output_port(0),
    //                 iiwa_controller->get_input_port_estimated_state());
    builder.Connect(plant_->get_state_output_port(iiwa_model_.model_instance),
                    iiwa_controller->get_input_port_estimated_state());

    // Add in feedforward torque.
    auto adder =
        builder.template AddSystem<systems::Adder>(2, num_iiwa_positions);
    builder.Connect(iiwa_controller->get_output_port_control(),
                    adder->get_input_port(0));
    builder.ExportInput(adder->get_input_port(1), "iiwa_feedforward_torque");
    builder.Connect(adder->get_output_port(), plant_->get_actuation_input_port(
                                                  iiwa_model_.model_instance));

    // Approximate desired state command from a discrete derivative of the
    // position command input port.
    auto desired_state_from_position = builder.template AddSystem<
        systems::StateInterpolatorWithDiscreteDerivative>(num_iiwa_positions,
                                                          plant_->time_step());
    desired_state_from_position->set_name("desired_state_from_position");
    builder.Connect(desired_state_from_position->get_output_port(),
                    iiwa_controller->get_input_port_desired_state());
    builder.Connect(iiwa_position->get_output_port(),
                    desired_state_from_position->get_input_port());

    // Export commanded torques:
    builder.ExportOutput(adder->get_output_port(), "iiwa_torque_commanded");
    builder.ExportOutput(adder->get_output_port(), "iiwa_torque_measured");
  }

  //Conveyor belt connections
  {
    // auto vector_source_system = builder.AddSystem(std::make_unique<multibody::conveyor_belt::ConveyorControl<double>>());
    // auto robot_plant = builder.AddSystem(std::make_unique<multibody::conveyor_belt::ConveyorPlant<double>>(frame_ids));
    auto belt_controller1 = builder.AddSystem(std::make_unique<multibody::conveyor_belt::ConveyorController<double>>(frame_ids1));
    // auto belt_controller2 = builder.AddSystem(std::make_unique<multibody::conveyor_belt::ConveyorController<double>>(frame_ids2));
    // auto belt_controller3 = builder.AddSystem(std::make_unique<multibody::conveyor_belt::ConveyorController<double>>(frame_ids3));


    //Velocity sending
    builder.Connect(belt_controller1->get_output_port(1),
                    plant_->get_actuation_input_port(conveyor_model_1_.model_instance));
    // builder.Connect(belt_controller2->get_output_port(1),
    //                 plant_->get_actuation_input_port(conveyor_model_2_.model_instance));
    // builder.Connect(belt_controller3->get_output_port(1),
    //                 plant_->get_actuation_input_port(conveyor_model_3_.model_instance));
  
    //Receive the states
    builder.Connect(plant_->get_state_output_port(conveyor_model_1_.model_instance),
                    belt_controller1->get_input_port(0)); 
    // //Receive the states
    // builder.Connect(plant_->get_state_output_port(conveyor_model_2_.model_instance),
    //                 belt_controller2->get_input_port(0)); 
    // //Receive the states
    // builder.Connect(plant_->get_state_output_port(conveyor_model_3_.model_instance),
    //                 belt_controller3->get_input_port(0)); 
  

    // Approximate desired state command from a discrete derivative of the
    // position command input port.
    auto desired_state_from_position_1 = builder.template AddSystem<
        systems::StateInterpolatorWithDiscreteDerivative>(3, plant_->time_step());
    desired_state_from_position_1->set_name("desired_state_from_position_belt_1");
    builder.Connect(belt_controller1->get_output_port(0),
                    desired_state_from_position_1->get_input_port());

    // // Approximate desired state command from a discrete derivative of the
    // // position command input port.
    // auto desired_state_from_position_2 = builder.template AddSystem<
    //     systems::StateInterpolatorWithDiscreteDerivative>(2, plant_->time_step());
    // desired_state_from_position_2->set_name("desired_state_from_position_belt_2");
    // builder.Connect(belt_controller2->get_output_port(0),
    //                 desired_state_from_position_2->get_input_port());
    // // Approximate desired state command from a discrete derivative of the
    // // position command input port.
    // auto desired_state_from_position_3 = builder.template AddSystem<
    //     systems::StateInterpolatorWithDiscreteDerivative>(2, plant_->time_step());
    // desired_state_from_position_3->set_name("desired_state_from_position_belt_3");
    // builder.Connect(belt_controller3->get_output_port(0),
    //                 desired_state_from_position_3->get_input_port());
  }

  {
    auto ee_controller = builder.template AddSystem<
        manipulation::yaskawa_conveyor_belt_dof1::EndEffectorPositionController>(
        manipulation::yaskawa_conveyor_belt_dof1::kEndEffectorLcmStatusPeriod, ee_kp_, ee_kd_);
    ee_controller->set_name("ee_controller");

    //Input accelerations to multibody plant - size 4
    builder.Connect(ee_controller->get_generalized_acceleration_output_port(),
                    plant_->get_actuation_input_port(wsg_model_.model_instance));

    //States sent from mbp to controller - size 8
    builder.Connect(plant_->get_state_output_port(wsg_model_.model_instance),
                    ee_controller->get_state_input_port());

    //State interpolator of only position of size 3
    builder.ExportInput(ee_controller->get_actuation_input_port(),
                        "ee_actuation");

    auto wsg_mbp_state_to_wsg_state = builder.template AddSystem(
        manipulation::yaskawa_conveyor_belt_dof1::MakeMultibodyStateToBeltStateSystem<double>());

    //States sent from mbp to wsgState - size 8
    builder.Connect(plant_->get_state_output_port(wsg_model_.model_instance),
                    wsg_mbp_state_to_wsg_state->get_input_port());

    //Should be size 8
    builder.ExportOutput(wsg_mbp_state_to_wsg_state->get_output_port(),
                         "ee_state_measured");
  }
//----------------------------------------------------------------------------------
  builder.ExportOutput(plant_->get_generalized_contact_forces_output_port(
                           iiwa_model_.model_instance),
                       "iiwa_torque_external");

  {  // RGB-D Cameras
    if (render_engines.size() > 0) {
      for (auto& pair : render_engines) {
        scene_graph_->AddRenderer(pair.first, std::move(pair.second));
      }
    } else {
      scene_graph_->AddRenderer(default_renderer_name_,
                                MakeRenderEngineVtk(RenderEngineVtkParams()));
    }

    for (const auto& info_pair : camera_information_) {
      std::string camera_name = "camera_" + info_pair.first;
      const CameraInformation& info = info_pair.second;

      const std::optional<geometry::FrameId> parent_body_id =
          plant_->GetBodyFrameIdIfExists(info.parent_frame->body().index());
      DRAKE_THROW_UNLESS(parent_body_id.has_value());
      const RigidTransform<double> X_PC =
          info.parent_frame->GetFixedPoseInBodyFrame() * info.X_PC;

      auto camera = builder.template AddSystem<systems::sensors::RgbdSensor>(
          parent_body_id.value(), X_PC, info.properties);
      builder.Connect(scene_graph_->get_query_output_port(),
                      camera->query_object_input_port());

      builder.ExportOutput(camera->color_image_output_port(),
                           camera_name + "_rgb_image");
      builder.ExportOutput(camera->depth_image_16U_output_port(),
                           camera_name + "_depth_image");
      builder.ExportOutput(camera->label_image_output_port(),
                           camera_name + "_label_image");
    }
  }

  builder.ExportOutput(scene_graph_->get_pose_bundle_output_port(),
                       "pose_bundle");
  builder.ExportOutput(plant_->get_contact_results_output_port(),
                       "contact_results");
  builder.ExportOutput(plant_->get_state_output_port(),
                       "plant_continuous_state");
  builder.ExportOutput(plant_->get_geometry_poses_output_port(),
                       "geometry_poses");

  builder.BuildInto(this);
  drake::log()->info("Finalize 12");

}

template <typename T>
VectorX<T> ManipulationStation<T>::GetIiwaPosition(
    const systems::Context<T>& station_context) const {

  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  drake::log()->info("GetIiwaPosition 13");

  return plant_->GetPositions(plant_context, iiwa_model_.model_instance);
}

template <typename T>
void ManipulationStation<T>::SetIiwaPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& q) const {
  const int num_iiwa_positions =
      plant_->num_positions(iiwa_model_.model_instance);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(q.size() == num_iiwa_positions);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetPositions(plant_context, &plant_state, iiwa_model_.model_instance,
                       q);

  // Set the position history in the state interpolator to match.
  const auto& state_from_position = dynamic_cast<
      const systems::StateInterpolatorWithDiscreteDerivative<double>&>(
      this->GetSubsystemByName("desired_state_from_position"));
  state_from_position.set_initial_position(
      &this->GetMutableSubsystemState(state_from_position, state), q);
  drake::log()->info("SetIiwaPosition 14");

}

template <typename T>
VectorX<T> ManipulationStation<T>::GetIiwaVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  drake::log()->info("GetIiwaVelocity 15");

  return plant_->GetVelocities(plant_context, iiwa_model_.model_instance);
}

template <typename T>
void ManipulationStation<T>::SetIiwaVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& v) const {
  const int num_iiwa_velocities =
      plant_->num_velocities(iiwa_model_.model_instance);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(v.size() == num_iiwa_velocities);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetVelocities(plant_context, &plant_state, iiwa_model_.model_instance,
                        v);
  drake::log()->info("SetIiwaVelocity 16");

}

template <typename T>
void ManipulationStation<T>::SetConveyorPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& q, const int num) const {
  const int num_belt_positions =
      plant_->num_positions(conveyor_model_1_.model_instance);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(q.size() == num_belt_positions);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  switch(num){
    case 1: {
      plant_->SetPositions(plant_context, &plant_state, conveyor_model_1_.model_instance, q);
      // Set the position history in the state interpolator to match.
      const auto& state_from_position1 = dynamic_cast<
          const systems::StateInterpolatorWithDiscreteDerivative<double>&>(
          this->GetSubsystemByName("desired_state_from_position_belt_1"));
      state_from_position1.set_initial_position(
          &this->GetMutableSubsystemState(state_from_position1, state), q);
      break;
    }
    case 2: {
      plant_->SetPositions(plant_context, &plant_state, conveyor_model_2_.model_instance, q);
      // Set the position history in the state interpolator to match.
      const auto& state_from_position2 = dynamic_cast<
          const systems::StateInterpolatorWithDiscreteDerivative<double>&>(
          this->GetSubsystemByName("desired_state_from_position_belt_2"));
      state_from_position2.set_initial_position(
          &this->GetMutableSubsystemState(state_from_position2, state), q);
      break;
    }
    case 3:{
      plant_->SetPositions(plant_context, &plant_state, conveyor_model_3_.model_instance, q);
      // Set the position history in the state interpolator to match.
      const auto& state_from_position3 = dynamic_cast<
          const systems::StateInterpolatorWithDiscreteDerivative<double>&>(
          this->GetSubsystemByName("desired_state_from_position_belt_3"));
      state_from_position3.set_initial_position(
          &this->GetMutableSubsystemState(state_from_position3, state), q);
      break;
    }
  }

  drake::log()->info("SetConveyorPosition 14");

}

template <typename T>
Vector3<T> ManipulationStation<T>::GetWsgPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector4<T> positions =
      plant_->GetPositions(plant_context, wsg_model_.model_instance);
  
  Vector3<T> posm;
      posm << positions(1) - positions(0), positions(2), positions(3);

  return posm;
}

template <typename T>
Vector3<T> ManipulationStation<T>::GetWsgVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector4<T> velocities =
      plant_->GetVelocities(plant_context, wsg_model_.model_instance);

  Vector3<T> vels;
    vels << (velocities(1) - velocities(0)), velocities(2), velocities(3);

  return vels;
}

template <typename T>
void ManipulationStation<T>::SetWsgPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::Vector3<T>>& q) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  // Since plant_->SetPositions has to take in a "const" of "positions" vector, 
  // I had to create a nonConstPos to assign it to the const positions
  Vector4<T> nonConstPos; 
  nonConstPos << -q(0) / 2, q(0) / 2, q(1), q(2);

  const Vector4<T> positions = nonConstPos; 
  plant_->SetPositions(plant_context, &plant_state, wsg_model_.model_instance,
                       positions);

  // Set the position history in the state interpolator to match.
  const auto& wsg_controller = dynamic_cast<
      const manipulation::yaskawa_conveyor_belt_dof1::EndEffectorPositionController&>(
      this->GetSubsystemByName("wsg_controller"));
  wsg_controller.set_initial_position(
      &this->GetMutableSubsystemState(wsg_controller, state), q);
}

template <typename T>
void ManipulationStation<T>::SetWsgVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::Vector3<T>>& v) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  // Since plant_->SetVelocties has to take in a "const" of "velocities" vector, 
  // I had to create a nonConstVel to assign it to the const velocities
  Vector4<T> nonConstVel;
  nonConstVel << -v(0) / 2, v(0) / 2, v(1), v(2);

  const Vector4<T> velocities =  nonConstVel;
  plant_->SetVelocities(plant_context, &plant_state, wsg_model_.model_instance,
                        velocities);
}

template <typename T>
std::vector<std::string> ManipulationStation<T>::get_camera_names() const {
  std::vector<std::string> names;
  names.reserve(camera_information_.size());
  for (const auto& info : camera_information_) {
    names.emplace_back(info.first);
  }
  drake::log()->info("get_camera_names 17");

  return names;
}

template <typename T>
void ManipulationStation<T>::SetWsgGains(const double kp, const double kd) {
  DRAKE_THROW_UNLESS(!plant_->is_finalized());
  DRAKE_THROW_UNLESS(kp >= 0 && kd >= 0);
  ee_kp_ = kp;
  ee_kd_ = kd;
}

template <typename T>
void ManipulationStation<T>::RegisterIiwaControllerModel(
    const std::string& model_path,
    const drake::multibody::ModelInstanceIndex iiwa_instance,
    const drake::multibody::Frame<T>& parent_frame,
    const drake::multibody::Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  // TODO(siyuan.feng@tri.global): We really only just need to make sure
  // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
  // from it to the world), and record that X_WP. However, the computation to
  // query X_WP given a partially constructed plant is not feasible at the
  // moment, so we are forcing the parent frame to be the world instead.
  DRAKE_THROW_UNLESS(parent_frame.name() == plant_->world_frame().name());

  iiwa_model_.model_path = model_path;
  iiwa_model_.parent_frame = &parent_frame;
  iiwa_model_.child_frame = &child_frame;
  iiwa_model_.X_PC = X_PC;

  iiwa_model_.model_instance = iiwa_instance;
  drake::log()->info("RegisterIiwaControllerModel 18");
}

template <typename T>
void ManipulationStation<T>::RegisterConveyorControllerModel(
    const int conveyor_model_num,
    const std::string& model_path,
    const drake::multibody::ModelInstanceIndex conveyor_instance,
    const drake::multibody::Frame<T>& parent_frame,
    const drake::multibody::Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  // TODO(siyuan.feng@tri.global): We really only just need to make sure
  // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
  // from it to the world), and record that X_WP. However, the computation to
  // query X_WP given a partially constructed plant is not feasible at the
  // moment, so we are forcing the parent frame to be the world instead.
  DRAKE_THROW_UNLESS(parent_frame.name() == plant_->world_frame().name());

  switch(conveyor_model_num){
    case 1:
      conveyor_model_1_.model_path = model_path;
      conveyor_model_1_.parent_frame = &parent_frame;
      conveyor_model_1_.child_frame = &child_frame;
      conveyor_model_1_.X_PC = X_PC;
      conveyor_model_1_.model_instance = conveyor_instance;
      break;
    case 2:
      conveyor_model_2_.model_path = model_path;
      conveyor_model_2_.parent_frame = &parent_frame;
      conveyor_model_2_.child_frame = &child_frame;
      conveyor_model_2_.X_PC = X_PC;
      conveyor_model_2_.model_instance = conveyor_instance;
      break;
    case 3:
      conveyor_model_3_.model_path = model_path;
      conveyor_model_3_.parent_frame = &parent_frame;
      conveyor_model_3_.child_frame = &child_frame;
      conveyor_model_3_.X_PC = X_PC;
      conveyor_model_3_.model_instance = conveyor_instance;
      break;
  }
  drake::log()->info("RegisterConveyorControllerModel 18");
}

template <typename T>
void ManipulationStation<T>::RegisterWsgControllerModel(
    const std::string& model_path,
    const drake::multibody::ModelInstanceIndex wsg_instance,
    const drake::multibody::Frame<T>& parent_frame,
    const drake::multibody::Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  wsg_model_.model_path = model_path;
  wsg_model_.parent_frame = &parent_frame;
  wsg_model_.child_frame = &child_frame;
  wsg_model_.X_PC = X_PC;

  wsg_model_.model_instance = wsg_instance;
}

template <typename T>
void ManipulationStation<T>::RegisterRgbdSensor(
    const std::string& name, const drake::multibody::Frame<T>& parent_frame,
    const RigidTransform<double>& X_PC,
    const geometry::render::DepthCameraProperties& properties) {
  CameraInformation info;
  info.parent_frame = &parent_frame;
  info.X_PC = X_PC;
  info.properties = properties;

  camera_information_[name] = info;
  drake::log()->info("RegisterRgbdSensor 19");

}

template <typename T>
std::map<std::string, RigidTransform<double>>
ManipulationStation<T>::GetStaticCameraPosesInWorld() const {
  std::map<std::string, RigidTransform<double>> static_camera_poses;

  for (const auto& info : camera_information_) {
    const auto& frame_P = *info.second.parent_frame;

    // TODO(siyuan.feng@tri.global): We really only just need to make sure
    // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
    // from it to the world). However, the computation to query X_WP given a
    // partially constructed plant is not feasible at the moment, so we are
    // looking for cameras that are directly attached to the world instead.
    const bool is_anchored =
        frame_P.body().index() == plant_->world_frame().body().index();
    if (is_anchored) {
      static_camera_poses.emplace(
          info.first,
          RigidTransform<double>(frame_P.GetFixedPoseInBodyFrame()) *
              info.second.X_PC);
    }
  }
  drake::log()->info("GetStaticCameraPosesInWorld 20");

  return static_camera_poses;
}

// Add default iiwa.
template <typename T>
void ManipulationStation<T>::AddDefaultIiwa(
    const IiwaCollisionModel collision_model) {
  std::string sdf_path;
  switch (collision_model) {
    case IiwaCollisionModel::kNoCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/yaskawa_description/urdf/"
            "yaskawa_no_collision.urdf"); 
            //"Motoman_GP25.urdf");
      break;
    case IiwaCollisionModel::kBoxCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/yaskawa_description/urdf/"
          "yaskawa_no_collision.urdf");
      break;
    default:
      throw std::domain_error("Unrecognized collision_model.");
  }
  const auto X_WI = RigidTransform<double>::Identity();
  auto iiwa_instance = internal::AddAndWeldModelFrom(
      sdf_path, "yaskawa", plant_->world_frame(), "arm_base", X_WI, plant_);
  RegisterIiwaControllerModel(
      sdf_path, iiwa_instance, plant_->world_frame(),
      plant_->GetFrameByName("arm_base", iiwa_instance), X_WI);
  drake::log()->info("AddDefaultIiwa 21");
}


template <typename T>
void ManipulationStation<T>::AddDefaultConveyor(
    drake::multibody::ModelInstanceIndex &model_idx, const int model_num, std::vector<geometry::FrameId> &frame_ids, const std::string& model_name) {

    const std::string sdf_path = FindResourceOrThrow(
        "drake/manipulation/models/conveyor_belt_description/sdf/"
        "conveyor_simple_robot.sdf");

    RigidTransform<double> X_BM(RotationMatrix<double>::MakeZRotation(0),Vector3d(1.7018, 2.32042, 0.8636-0.1375));
                                  //Vector3d(1.7, 2, 0.4));//0.8));//-0.8636)); //X value is 0.82042m (2-8.3') + 1.5m (addition)
                                  //(offset dist b/w robot center and beginning of conveyor + center of conveyor)

    model_idx = internal::AddAndWeldModelFrom(sdf_path, model_name, plant_->world_frame(),
                                                      "base_link", X_BM, plant_);

    RegisterConveyorControllerModel(
        model_num, sdf_path, model_idx, plant_->world_frame(),
        plant_->GetFrameByName("base_link", model_idx), X_BM);

    //Frame_ids size is 4 for 4 robot links
    for(int i = 0; i < 4; i++) {
      frame_ids.push_back({plant_->GetBodyFrameIdOrThrow(plant_->GetBodyIndices(model_idx)[i])});
    }
}
  
// Add default wsg.
template <typename T>
void ManipulationStation<T>::AddDefaultWsg() {

  const std::string sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/yaskawa_end_effector_description/urdf/end_effector_3.urdf");

  const drake::multibody::Frame<T>& link6 =
      plant_->GetFrameByName("ee_mount", iiwa_model_.model_instance);
  const RigidTransform<double> X_6G(RollPitchYaw<double>(0, 0, 0),
                                    Vector3d(0, 0, 0.114));
  auto wsg_instance = internal::AddAndWeldModelFrom(sdf_path, "gripper", link6,
                                                    "ee_base", X_6G, plant_);
  RegisterWsgControllerModel(sdf_path, wsg_instance, link6,
                             plant_->GetFrameByName("ee_base", wsg_instance),
                             X_6G);
  drake::log()->info("add default wsg");

}




}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

// TODO(russt): Support at least NONSYMBOLIC_SCALARS.  See #9573.
//   (and don't forget to include default_scalars.h)
template class ::drake::examples::manipulation_station::ManipulationStation<double>;
