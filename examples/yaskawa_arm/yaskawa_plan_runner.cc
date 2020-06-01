/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages contraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <memory>

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/yaskawa_arm/yaskawa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_ddp_traj.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

#define use_Solver 0

namespace drake {
namespace examples {
namespace yaskawa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const char* const kLcmDDPChannel = "DDP";
const int kNumJoints = 6;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class RobotPlanRunner {
 public:
  /// tree is aliased
  explicit RobotPlanRunner(const RigidBodyTree<double>& tree)
      : tree_(tree), plan_number_(0) {
    VerifyYaskawaTree(tree);
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);
    lcm_.subscribe(kLcmPlanChannel,
                    &RobotPlanRunner::HandlePlan, this);
    lcm_.subscribe(kLcmStopChannel,
                    &RobotPlanRunner::HandleStop, this);
    #if use_Solver
    lcm_.subscribe(kLcmDDPChannel,
                    &RobotPlanRunner::HandleDDP, this);
    #endif
  }

  void Run() {
    int cur_plan_number = plan_number_;
    int64_t cur_time_us = -1;
    int64_t start_time_us = -1;
    
    has_active_plan_ = false;
    cur_traj_idx_ = 0;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.utime = cur_time_us;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { }

      #if use_Solver
        //This is using DDP to solve for the trajectory
        cur_time_us = iiwa_status_.utime;

        if (has_active_plan_) {
          if (plan_number_ != cur_plan_number) {
            drake::log()->info("Starting new plan.");
            // start_time_us = cur_time_us;
            cur_plan_number = plan_number_;
            cur_traj_idx_ = 0;
          }

          // const double cur_traj_time_s =
          //     static_cast<double>(cur_time_us - start_time_us) / 1e6;

          // if new traj time step reached, publish new command
          iiwa_command.utime = iiwa_status_.utime;

          if (ddp_traj_.dim_torques) {
            drake::log()->info("Yaskawa arm should not receive torques. Should only be position control");
            DRAKE_DEMAND(false); //Kills code 
            iiwa_command.joint_position = iiwa_status_.joint_position_measured;
            for (int i=0; i<kNumJoints; i++) {
                iiwa_command.joint_torque[i] = 
                    ddp_traj_.torques[cur_traj_idx_][i];
            }
          } else {
            for (int i=0; i<kNumJoints; i++) { 
                iiwa_command.joint_position[i] = 
                    ddp_traj_.states[cur_traj_idx_][i];
            }
          }
          lcm_.publish(kLcmCommandChannel, &iiwa_command);
          cur_traj_idx_++;
        }

        // Checks to see if there are any more steps in the trajectory
        if (cur_traj_idx_ >= ddp_traj_.n_time_steps) {
            if(has_active_plan_){ drake::log()->info("Current plan completed. Waiting for new plan\n"); }
            has_active_plan_ = false;     
            
            //TODO: Send message telling that movement is finished
            // lcmt_generic_string_msg plan_status;
            // plan_status.msg = "Finished";
            // lcm_.publish(kExecutionStatusChannel, &plan_status);
        }

      #else
        //This mode is normal Inverse Kinematics planner provided by drake
        cur_time_us = iiwa_status_.utime;

        if (plan_) {
          if (plan_number_ != cur_plan_number) {
            drake::log()->info("Starting new plan.");
            start_time_us = cur_time_us;
            cur_plan_number = plan_number_;
          }

          const double cur_traj_time_s =
              static_cast<double>(cur_time_us - start_time_us) / 1e6;
          const auto desired_next = plan_->value(cur_traj_time_s);

          iiwa_command.utime = iiwa_status_.utime;

          for (int joint = 0; joint < kNumJoints; joint++) {
            iiwa_command.joint_position[joint] = desired_next(joint);
          }

          lcm_.publish(kLcmCommandChannel, &iiwa_command);
        }
      #endif
    }
  }

 private:
  void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    drake::log()->info("handler being called in plan runner");
    iiwa_status_ = *status;
  }

  void HandlePlan(const lcm::ReceiveBuffer*, const std::string&,
                  const robotlocomotion::robot_plan_t* plan) {
    drake::log()->info("New plan received.");

    if (iiwa_status_.utime == -1) {
      drake::log()->info("Discarding plan, no status message received yet");
      return;
    } else if (plan->num_states < 2) {
      drake::log()->info("Discarding plan, Not enough knot points.");
      return;
    }

    std::vector<Eigen::MatrixXd> knots(plan->num_states,
                                       Eigen::MatrixXd::Zero(kNumJoints, 1));
    std::map<std::string, int> name_to_idx =
        tree_.computePositionNameToIndexMap();
    for (int i = 0; i < plan->num_states; ++i) {
      const auto& state = plan->plan[i];
      for (int j = 0; j < state.num_joints; ++j) {
        if (name_to_idx.count(state.joint_name[j]) == 0) {
          continue;
        }
        // Treat the matrix at knots[i] as a column vector.
        if (i == 0) {
          // Always start moving from the position which we're
          // currently commanding.
          DRAKE_DEMAND(iiwa_status_.utime != -1);
          knots[0](name_to_idx[state.joint_name[j]], 0) =
              iiwa_status_.joint_position_commanded[j];
        } else {
          knots[i](name_to_idx[state.joint_name[j]], 0) =
              state.joint_position[j];
        }
      }
    }

    for (int i = 0; i < plan->num_states; ++i) {
      drake::log()->info(knots[i] );
    }

    std::vector<double> input_time;
    for (int k = 0; k < static_cast<int>(plan->plan.size()); ++k) {
      input_time.push_back(plan->plan[k].utime / 1e6);
    }
    const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints, 1);
    plan_.reset(new PiecewisePolynomial<double>(
        PiecewisePolynomial<double>::Cubic(input_time, knots,
                                           knot_dot, knot_dot)));
    ++plan_number_;
  }

  void HandleStop(const lcm::ReceiveBuffer*, const std::string&,
                    const robotlocomotion::robot_plan_t*) {
    drake::log()->info("Received stop command. Discarding plan.");
    plan_.reset();
  }

  void HandleDDP(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_ddp_traj* plan) {
    drake::log()->info(" trajectory received");
    ddp_traj_ = *plan;
    has_active_plan_ = true;
    plan_number_++;
  }


  //Private variables:
  lcmt_ddp_traj ddp_traj_;
  bool has_active_plan_;
  int cur_traj_idx_;


  lcm::LCM lcm_;
  const RigidBodyTree<double>& tree_;
  int plan_number_{};
  std::unique_ptr<PiecewisePolynomial<double>> plan_;
  lcmt_iiwa_status iiwa_status_;
};

int do_main() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/manipulation/models/yaskawa_description/urdf/"
                          "yaskawa_no_collision.urdf"),
      multibody::joints::kFixed, tree.get());

  RobotPlanRunner runner(*tree);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace yaskawa_arm
}  // namespace examples
}  // namespace drake


int main() {
  return drake::examples::yaskawa_arm::do_main();
}
