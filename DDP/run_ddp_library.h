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
#include "drake/lcmt_iiwa_status.hpp"
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/lcmt_ddp_traj.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/yaskawa_arm/yaskawa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/lcmt_generic_string_msg.hpp"



/* DDP trajectory generation */
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <string>
#include <list>

#include "drake/DDP/config.h"
// #include "drake/DDP/spline.h"  //I don't think this file is being used
#include "drake/DDP/iLQR_solver.h"
// #include "drake/DDP_traj_gen/udpsolver.h"
#include "drake/DDP/yaskawa_model.h"
#define UDP_TRAJ_DIR "~/drake/DDP/trajectory/"

using namespace std;
using namespace Eigen;

#define useILQRSolver 1
#define useUDPSolver 0
/* DDP trajectory generation */

static std::list< const char*> gs_fileName;
static std::list< std::string > gs_fileName_string;
namespace real_lcm = lcm; //Need this to differentiate from drake LCM

namespace drake {
namespace examples {
namespace yaskawa_arm {


class RunDDPLibrary
{
 public:

  void RunUDP(stateVec_t xinit, stateVec_t xgoal);

//   void saveVector(const Eigen::MatrixXd & _vec, const char * _name);

//   void saveValue(double _value, const char * _name);
 
//   void clean_file(const char * _file_name, std::string & _ret_file);
 
 private:
  real_lcm::LCM lcm_;

  lcmt_ddp_traj ddp_traj_;

  //UDP parameters
  stateVecTab_t joint_state_traj;
  commandVecTab_t torque_traj;
  stateVecTab_t joint_state_traj_interp;
  commandVecTab_t torque_traj_interp;
  // unsigned int traj_knot_number_ = 0;
};


}  // namespace yaskawa_arm
}  // namespace examples
}  // namespace drake