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
#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_position_controller.h"
#include "drake/manipulation/yaskawa_conveyor_belt_dof1/conveyor_belt_dof1_lcm.h"

namespace real_lcm = lcm;

namespace drake {
namespace examples{
namespace yaskawa_arm_runner{

class PrimitiveExecutor
{
public:
    PrimitiveExecutor();

    //Define actions that the arm can take
    int action_Collect(const double time);
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

    RigidBodyTree<double> tree_;

    lcmt_yaskawa_ee_status ee_status_;
    bool ee_update_;

    lcmt_iiwa_status iiwa_status_;
    bool yaskawa_update_;

    Eigen::Vector3d object_position_;
    Eigen::Quaterniond object_quaternion_;
    bool object_update_;

    void HandleStatusYaskawa(const real_lcm::ReceiveBuffer*, const std::string&, const lcmt_iiwa_status* status);
    void HandleStatusGripper(const real_lcm::ReceiveBuffer*, const std::string&, const lcmt_yaskawa_ee_status* status);
    void HandleStatusPackage(const real_lcm::ReceiveBuffer*, const std::string&, const bot_core::robot_state_t* status);

    bool is_at_desired_position();
    bool is_whisker_down();
    bool is_carrying();
    bool is_at_shelf();
    bool is_at_belt();
    bool is_moving();
    bool is_pickable();
    bool is_at_package_location();

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
