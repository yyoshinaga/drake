#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Tests that yaskawa_mesh_collision.urdf can be parsed into a plant.
GTEST_TEST(PBDescriptionTest, TestMeshCollisionModelLoadTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/PB_Layout_V2_description/sdf/"
      "PB_Layout_V2.sdf"));

  multibody::MultibodyPlant<double> plant;
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);
  plant.Finalize();

  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Finally, there is a model instance for the gripper.
  EXPECT_EQ(plant.num_model_instances(), 3);

  EXPECT_EQ(plant.num_positions(), 2);
  EXPECT_EQ(plant.num_velocities(), 2);

  ASSERT_EQ(plant.num_bodies(), 9);
  const std::vector<std::string> expected_body_names {
      "" /* Ignore the world body name */,
      "arm_base",
      "f_link",
      "s_link",
      "l_link",
      "u_link",
      "r_link",
      "b_link",
      "t_link",
      "ee_mount"};
  for (multibody::BodyIndex i{1}; i < plant.num_bodies(); ++i) {
    EXPECT_THAT(plant.get_body(i).name(),
                testing::EndsWith(expected_body_names.at(i)));
  }
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
