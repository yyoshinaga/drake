#ifndef PRIMITIVE_EXECUTOR_H
#define PRIMITIVE_EXECUTOR_H

#include "lcm/lcm-cpp.hpp"
#include <vector>
#include <gflags/gflags.h>
#include "robotlocomotion/robot_plan_t.hpp"
#include <Eigen/Geometry> 
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"

#include "drake/examples/yaskawa_arm/yaskawa_common.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

//Yaskawa ee 
#include "drake/lcmt_yaskawa_ee_command.hpp"
#include "drake/lcmt_yaskawa_ee_status.hpp"
#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_constants.h"
#include "drake/manipulation/yaskawa_conveyor_belt_dof1/new_belt_position_controller.h"
#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_lcm.h"

//DDP
//#include "drake/DDP/run_ddp_library.h"

namespace real_lcm = lcm;

namespace drake {
namespace examples{
namespace yaskawa_arm_runner{

class PrimitiveExecutor
{
public:
    PrimitiveExecutor();

    //Define actions that the arm can take
    int action_GoHome();
    int action_Hault();                     //For ee belt
    int action_Collect(const double time);  //TODO: Get rid of timed movement. Stopping should occur with a command.
    int action_Release(const double time);
    int action_WhiskersUp();
    int action_WhiskersDown();
    int action_GoToPoint(const Eigen::Vector3d position, const Eigen::Vector3d orientation, const double time);

    Eigen::Vector3d getObjectLocation();
    Eigen::Quaterniond getObjectQuat();

private:
    real_lcm::LCM lcm_;
    std::string yaskawa_urdf_;
    std::string ee_urdf_;
    math::RigidTransform<double> ee_pose;
    RigidBodyTree<double> tree_;

    //States of the robot: 

    // Since collision detection is difficult, the code assumes that once it calls
    // action_Collect(), this variable will turn true. Once it calls action_Release(),
    // this variable will turn false. Default is false.
    bool has_package; 

    //Statuses
    lcmt_iiwa_status iiwa_status_;
    bool yaskawa_update_;

    lcmt_yaskawa_ee_status ee_status_;

    //Locks the end-effector into only executing one action at a time.
    bool ee_update_;    

    Eigen::Vector3d object_position_;
    Eigen::Quaterniond object_quaternion_;
    bool object_update_;

    //Subscriber functions
    void HandleStatusYaskawa(const real_lcm::ReceiveBuffer*, const std::string&, const lcmt_iiwa_status* status);
    void HandleStatusGripper(const real_lcm::ReceiveBuffer*, const std::string&, const lcmt_yaskawa_ee_status* status);
    void HandleStatusPackage(const real_lcm::ReceiveBuffer*, const std::string&, const bot_core::robot_state_t* status);

    // This function checks to see whether the yaskawa arm's end effector frame is 
    // at its desired pose. Includes translation and orientation. 
    bool is_at_desired_position(VectorX<double> des_pos);

    // This function compares status' whisker angle to the FLAGS_whiskers_up_angle. 
    // The up position is set to 0 deg (0 rad)
    bool are_whiskers_up();

    // This function compares status' whisker angle to the FLAGS_whiskers_down_angle. 
    // Currently the down position is set to -90deg (-1.57 rad)
    bool are_whiskers_down();

    // This function checks to see if the end effector is carrying a box
    bool is_carrying();

    // This function checks to see if the end effector is by the shelf
    bool is_at_shelf();

    // This function checks to see if the end effector is at the belt location
    bool is_at_belt();

    // This function checks to see if the arm is currently moving
    bool is_moving();

    // This function checks to see if the box is too large to be picked up
    // by the end effector
    bool is_pickable();

    // This function checks to see if the end effector is positioned correctly at
    // the belt and moved to the correct spot at the belt to pick up the package. 
    // The package could potentially be towards the right side of the belt or the 
    // left side of the belt. Adds amount x to the belt location.
    bool is_at_package_location(double x);

    //Prints checker functions
    void printer(bool desPos, bool whiskUp, bool whiskDwn, bool carry, bool atShlf,  
                 bool atBlt, bool isMvng, bool isPckble, bool atPkgLoc);

    multibody::MultibodyPlant<double> plant_;
    std::unique_ptr<systems::Context<double>> context_;
    std::vector<std::string> joint_names_;
    int status_count_{0};

    drake::multibody::ModelInstanceIndex yaskawa_model_idx;
    drake::multibody::ModelInstanceIndex ee_model_idx;


};

}
}
}


#endif
