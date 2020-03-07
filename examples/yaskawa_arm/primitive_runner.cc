// Create end effector executor that doesn't read from command line interface
#include <iostream>
#include "drake/examples/yaskawa_arm/primitive_executor.h"
#include "lcm/lcm-cpp.hpp"
// #include "drake/common/text_logging_gflags.h"
// #include "drake/manipulation/planner/constraint_relaxing_ik.h"
// #include "drake/math/rigid_transform.h"
// #include "drake/math/roll_pitch_yaw.h"
// #include "drake/math/rotation_matrix.h"

int main(int , char** ) {
    drake::examples::yaskawa_arm_runner::PrimitiveExecutor p_executor;

    // Eigen::Vector3d object_location = p_executor.getObjectLocation();
    p_executor.action_Collect(1);
    drake::log()->info("action: Collect");
    // // p_executor.action_GoToPoint(Eigen::Vector3d(.75, 0, .15), Eigen::Vector3d(0.0, 1.57, 0.0));
    // std::cout << object_location << std::endl;

    // object_location(0) -= .02;
    // object_location(2) += .2;
    // std::cout << object_location << std::endl;
    // p_executor.action_GoToPoint(object_location, Eigen::Vector3d(0.0, 1.57, 0.0));
    // // p_executor.action_GoToPoint(object_location, Eigen::Vector3d(0.0, 1.57, 0.0));

    // object_location(2) -= .1;
    // std::cout << object_location << std::endl;
    // p_executor.action_GoToPoint(object_location, Eigen::Vector3d(0.0, 1.57, 0.0));
    // p_executor.action_Grip(40, 17);
    // object_location(2) += .2;
    // object_location(0) -= .1;
    // std::cout << object_location << std::endl;
    // p_executor.action_GoToPoint(object_location, Eigen::Vector3d(0.0, 1.57, 0.0));
    // p_executor.action_Grip(80, 40);

}
