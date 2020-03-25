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

    // p_executor.action_GoHome();

    drake::log()->info("action: goToPoint");
    p_executor.action_GoToPoint(Eigen::Vector3d(1.0, -0.2, 0.7), Eigen::Vector3d(0.0, 0.0,1.57),8.0);
    p_executor.action_GoToPoint(Eigen::Vector3d(1.65, 0.1, 0.9), Eigen::Vector3d(0.0, 0.0,1.57),3.0);

    drake::log()->info("action: Collect");
    DRAKE_DEMAND(p_executor.action_Collect(2.0) ? true : false);

    drake::log()->info("action: WhiskersDown");
    DRAKE_DEMAND(p_executor.action_WhiskersDown());

    drake::log()->info("action: goToPoint");
    p_executor.action_GoToPoint(Eigen::Vector3d(1.0, -0.1, 1.7), Eigen::Vector3d(0.0, 0.0,1.65),5.0);
    p_executor.action_GoToPoint(Eigen::Vector3d(0.6, 0.75, 1.73), Eigen::Vector3d(0.0, 0.4,1.65),5.0);

    drake::log()->info("action: WhiskersUp");
    DRAKE_DEMAND(p_executor.action_WhiskersUp());

    drake::log()->info("action: Release");
    DRAKE_DEMAND(p_executor.action_Release(2.0) ? true : false);


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
