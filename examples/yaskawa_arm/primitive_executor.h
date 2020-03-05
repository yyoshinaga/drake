#ifndef PRIMITIVE_EXECUTOR_H
#define PRIMITIVE_EXECUTOR_H

#include "lcm/lcm-cpp.hpp"
#include <vector>
#include <gflags/gflags.h>
#include "robotlocomotion/robot_plan_t.hpp"

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

namespace drake {
namespace examples{
namespace yaskawa_arm_runner{


class PrimitiveExecutor
{
public:
    PrimitiveExecutor();

    int action_Grip(double width, double force, double time = 5.0);
    int action_GoToPoint(Eigen::Vector3d position, Eigen::Vector3d orientation, double time = 7.0);
    std::vector<Eigen::Vector3d> getObjectLocations();
    std::vector<Eigen::Quaterniond> getObjectQuats();
    // Eigen::VectorXd getEEInfo();

private:
    lcm::LCM lcm_;
    std::string yaskawa_urdf_;
    std::string ee_urdf_;

    RigidBodyTree<double> tree_;

    lcmt_yaskawa_ee_status ee_status_;
    bool ee_update_;

    lcmt_iiwa_status iiwa_status_;
    bool iiwa_update_;

    std::vector<Eigen::Vector3d> object_positions_;
    std::vector<Eigen::Quaterniond> object_quaternions_;
    bool object_update_;

    void HandleStatusYaskawa(const lcm::ReceiveBuffer*, const std::string&, const lcmt_iiwa_status* status);
    void HandleStatusGripper(const lcm::ReceiveBuffer*, const std::string&, const lcmt_yaskawa_ee_status* status);
    void HandleStatusPackage(const lcm::ReceiveBuffer*, const std::string&, const bot_core::robot_state_t* status);

    multibody::MultibodyPlant<double> plant_;
    std::unique_ptr<systems::Context<double>> context_;
    std::vector<std::string> joint_names_;
    int status_count_{0};

};

}
}
}


#endif
