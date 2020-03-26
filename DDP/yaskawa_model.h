// #pragma once

#ifndef YASKAWAARM_H
#define YASKAWAARM_H

#include "drake/DDP/config.h"
#include "drake/DDP/yaskawa_cost_function.h"

// #include "drake/common/drake_path.h"
// #include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
// #include "drake/common/trajectories/piecewise_polynomial.h"
// #include "drake/examples/kuka_yaskawa_arm/yaskawa_common.h"
// #include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

#include <cstdio>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

// #include <mutex>
// std::mutex mtx;

#define pi 3.141592653

#ifndef DEBUG_YASKAWA_ARM
#define DEBUG_YASKAWA_ARM 1
#else
    #if PREFIX1(DEBUG_KUKA_ARM)==1
    #define DEBUG_KUKA_ARM 1
    #endif
#endif

#define TRACE_YASKAWA_ARM(x) do { if (DEBUG_YASKAWA_ARM) printf(x);} while (0)

using namespace Eigen;
using namespace std;
namespace real_lcm = lcm;

// using drake::manipulation::planner::ConstraintRelaxingIk;
// using drae::manipulation::kuka_yaskawa::kyaskawaArmNumJoints;
// using drake::manipulation::util::WorldSimTreeBuilder;
// using drake::manipulation::util::ModelInstanceInfo;
// using drake::systems::RigidBodyPlant;

namespace drake {
namespace examples {
namespace yaskawa_arm {
using multibody::MultibodyPlant;

class YaskawaModel
{
public:
    YaskawaModel(double& yaskawa_dt, unsigned int& yaskawa_N, stateVec_t& yaskawa_xgoal);
    YaskawaModel(double& yaskawa_dt, unsigned int& yaskawa_N, stateVec_t& yaskawa_xgoal, std::unique_ptr<MultibodyPlant<double>>& plant_);
    ~YaskawaModel(){};
private:
    real_lcm::LCM lcm_;
    lcmt_iiwa_status iiwa_status_;

protected:
    // attributes
    unsigned int stateNb;
    unsigned int commandNb;
    commandVec_t lowerCommandBounds;
    commandVec_t upperCommandBounds;

    stateMat_t fx;
    stateTens_t fxx;
    stateR_commandC_t fu;
    stateR_commandC_commandD_t fuu;
    stateR_stateC_commandD_t fxu;
    stateR_commandC_stateD_t fux;

    stateMatTab_t fxList;
    stateR_commandC_tab_t fuList;
    stateTensTab_t fxxList;
    stateTensTab_t fxuList;
    stateR_commandC_Tens_t fuuList;

public:
    struct timeprofile
    {
        double time_period1, time_period2, time_period3, time_period4;
        unsigned int counter0_, counter1_, counter2_;
    };

private:
    double dt;
    unsigned int N;
    bool initial_phase_flag_;
    struct timeprofile finalTimeProfile;
    struct timeval tbegin_period, tend_period, tbegin_period4, tend_period4; //tbegin_period2, tend_period2, tbegin_period3, tend_period3, 

public:
    static const double mc, mp, l, g;
    
    //######
    unsigned int globalcnt;
    //######
    
    stateVec_t xgoal;
private:
    
    stateMat_half_t H, C; // inertial, corillois dynamics
    stateVec_half_t G; // gravity? what is this?
    stateR_half_commandC_t Bu; //input mapping
    stateVec_half_t velocity;
    stateVec_half_t accel;
    stateVec_t Xdot_new;
    stateVec_half_t vd;
    stateVecTab_half_t vd_thread;
    stateVecTab_t Xdot_new_thread;

    stateVec_t Xdot1, Xdot2, Xdot3, Xdot4;
    stateMat_t A1, A2, A3, A4, IdentityMat;
    stateR_commandC_t B1, B2, B3, B4;
    stateVec_t Xp, Xp1, Xp2, Xp3, Xp4, Xm, Xm1, Xm2, Xm3, Xm4;
    
    bool debugging_print;
    stateMat_t AA;
    stateR_commandC_t BB;
    stateMatTab_t A_temp;
    stateR_commandC_tab_t B_temp;
    
    // std::unique_ptr<RigidBodyTree<double>> totalTree_{nullptr};
    std::unique_ptr<MultibodyPlant<double>> robot_thread_{nullptr};
    std::unique_ptr<systems::Context<double>> context_;

    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    std::vector<Eigen::VectorXd> q_thread, qd_thread;
protected:
    // methods
public:
    stateVec_t yaskawa_arm_dynamics(const stateVec_t& X, const commandVec_t& tau);

    //Subscriber function
    void HandleStatusYaskawa(const real_lcm::ReceiveBuffer*, const std::string&, const lcmt_iiwa_status* status);

    void yaskawa_arm_dyn_cst_ilqr(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList, CostFunctionYaskawa*& costFunction);
    void yaskawa_arm_dyn_cst_min_output(const int& nargout, const double& dt, const stateVec_t& xList_curr, const commandVec_t& uList_curr,  const bool& isUNan, stateVec_t& xList_next, CostFunctionYaskawa*& costFunction);
    void yaskawa_arm_dyn_cst_udp(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList, CostFunctionYaskawa*& costFunction);
    // void kuka_arm_dyn_cst_v3(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList, stateTensTab_t& fxxList, stateTensTab_t& fxuList, stateR_commandC_Tens_t& fuuList, CostFunctionYaskawa*& costFunction);
    stateVec_t update(const int& nargout, const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B);
    void grad(const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B);
    void hessian(const stateVec_t& X, const commandVec_t& U, stateTens_t& fxx_p, stateR_stateC_commandD_t& fxu_p, stateR_commandC_commandD_t& fuu_p);    
    struct timeprofile getFinalTimeProfile();

    unsigned int getStateNb();
    unsigned int getCommandNb();
    commandVec_t& getLowerCommandBounds();
    commandVec_t& getUpperCommandBounds();
    stateMatTab_t& getfxList();
    stateR_commandC_tab_t& getfuList();
private:
protected:
        // accessors //
public:
};

}  // namespace yaskawa_arm
}  // namespace examples
}  // namespace drake

#endif // YASKAWAARM_H
