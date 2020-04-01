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

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

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
namespace {

const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const int32_t kNumJoints = 6;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

using manipulation::planner::ConstraintRelaxingIk;
using manipulation::yaskawa::kYaskawaArmNumJoints;

class RobotPlanRunner {
  public:
  void RunUDP(stateVec_t xinit, stateVec_t xgoal) {
    struct timeval tbegin,tend;
    double texec = 0.0;

    double dt = TimeStep;
    unsigned int N = NumberofKnotPt;
    double tolFun = 1e-5;//1e-5;//relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad = 1e-5;//relaxing default value: 1e-10; - gradient exit criteria
    unsigned int iterMax = 100; //100;
    #if useILQRSolver
        ILQRSolver::traj lastTraj;
        //=============================================
        // Build wholebody and pass over to yaskawa
        const char* const kYaskawaUrdf = "drake/manipulation/models/yaskawa_description/urdf/yaskawa_no_collision.urdf";
        const char* const kEEUrdf = "drake/manipulation/models/yaskawa_end_effector_description/urdf/end_effector_3.urdf";
 
        // std::string urdf_;
        // RigidBodyTree<double> yaskawaTree_;
        // std::unique_ptr<RigidBodyTree<double>> totalTree_;
        // totalTree_ = std::make_unique<RigidBodyTree<double>>();

        // std::unique_ptr<WorldSimTreeBuilder<double>> tree_builder; 
        // std::unique_ptr<RigidBodyPlant<double>> plant;

        // tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
        // tree_builder->StoreDrakeModel("iiwa", kIiwaUrdf);
        // tree_builder->StoreDrakeModel("wsg", schunkPath);
        // tree_builder->StoreDrakeModel("connector", connectorPath);

        // const Eigen::Vector3d kRobotBase(0, 0, 0);

        // ModelInstanceInfo<double> iiwa_instance, connector_instance, wsg_instance;

        // int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
        // iiwa_instance = tree_builder->get_model_info_for_instance(id);

        // id = tree_builder->AddModelInstanceToFrame(
        //     "connector", tree_builder->tree().findFrame("iiwa_frame_ee"),
        //     drake::multibody::joints::kFixed);
        // connector_instance = tree_builder->get_model_info_for_instance(id);
        
        // //The schunk will be 0.09 above the kuka end effector -> change number by going to the urdf of iiwa7
        // id = tree_builder->AddModelInstanceToFrame(
        //     "wsg", tree_builder->tree().findFrame("connector_frame"),
        //     drake::multibody::joints::kFixed);
        // wsg_instance = tree_builder->get_model_info_for_instance(id);

        // totalTree_ = tree_builder->Build();
        // Build wholebody and pass over to kukaArm
        //=============================================

        auto plant_ = std::make_unique<multibody::MultibodyPlant<double>>(0.0);

        const math::RigidTransform<double> X_WI = math::RigidTransform<double>::Identity();
        const math::RigidTransform<double> X_6G(math::RollPitchYaw<double>(0, 0, 0),
                                            Eigen::Vector3d(0, 0, 0.114));

        DRAKE_THROW_UNLESS(!plant_->HasModelInstanceNamed("yaskawa"));
        DRAKE_THROW_UNLESS(!plant_->HasModelInstanceNamed("gripper"));

        std::string urdf_ = "";
        drake::multibody::Parser parser(plant_.get());

        const std::string yaskawa_urdf_ =
            (!urdf_.empty() ? urdf_ : FindResourceOrThrow(kYaskawaUrdf));
        const std::string ee_urdf_ =
            (!urdf_.empty() ? urdf_ : FindResourceOrThrow(kEEUrdf));

        drake::multibody::ModelInstanceIndex yaskawa_model_idx =
            parser.AddModelFromFile(yaskawa_urdf_, "yaskawa");
        drake::multibody::ModelInstanceIndex ee_model_idx =
            parser.AddModelFromFile(ee_urdf_, "gripper");   

        // Add EE to model
        const drake::multibody::Frame<double>& ee_frame =
            plant_->GetFrameByName("ee_mount", yaskawa_model_idx);

        //Weld models
        plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("arm_base", yaskawa_model_idx), X_WI);
        plant_->WeldFrames(ee_frame, plant_->GetFrameByName("ee_base", ee_model_idx), X_6G);

        plant_->Finalize();
        drake::log()->info("Plant finalized");

        //============================================
        #if WHOLE_BODY
          YaskawaModel yaskawaArm(dt, N, xgoal, plant_);
        #else
          YaskawaModel yaskawaArm(dt, N, xgoal);
        #endif
        CostFunctionYaskawa yaskawaCost;
        ILQRSolver testSolverYaskawa(yaskawaArm,yaskawaCost,ENABLE_FULLDDP,ENABLE_QPBOX);
        testSolverYaskawa.firstInitSolver(xinit, xgoal, N, dt, iterMax, tolFun, tolGrad);    
    #endif
    #if useUDPSolver
        double scale = 1e-4;//[To be optimized]
        YaskawaModel yaskawaArm(dt, N, xgoal);
        YaskawaModel::timeprofile finalTimeProfile;
        UDPSolver::traj lastTraj;
        CostFunctionYaskawa yaskawaCost;
        UDPSolver testSolverYaskawa(yaskawaArm,yaskawaCost,ENABLE_FULLDDP,ENABLE_QPBOX);
        testSolverYaskawa.firstInitSolver(xinit, xgoal, N, dt, scale, iterMax, tolFun, tolGrad);    
    #endif

    // run one or multiple times and then average
    unsigned int Num_run = 1;
    gettimeofday(&tbegin,NULL);
    for(unsigned int i=0;i<Num_run;i++) testSolverYaskawa.solveTrajectory();
    gettimeofday(&tend,NULL);

    lastTraj = testSolverYaskawa.getLastSolvedTrajectory();
    #if useUDPSolver
      finalTimeProfile = YaskawaModel.getFinalTimeProfile();
    #endif
    joint_state_traj.resize(N+1);
    joint_state_traj_interp.resize(N*InterpolationScale+1);
    for(unsigned int i=0;i<=N;i++){
      joint_state_traj[i] = lastTraj.xList[i];
    }
    torque_traj = lastTraj.uList;

    //linear interpolation to 1ms
    for(unsigned int i=0;i<stateSize;i++){
      for(unsigned int j=0;j<N*InterpolationScale;j++){
       unsigned int index = j/10;
       joint_state_traj_interp[j](i,0) =  joint_state_traj[index](i,0) + (static_cast<double>(j)-static_cast<double>(index*10.0))*(joint_state_traj[index+1](i,0) - joint_state_traj[index](i,0))/10.0;
      }
      joint_state_traj_interp[N*InterpolationScale](i,0) = joint_state_traj[N](i,0);
    }

    texec=(static_cast<double>(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec /= Num_run;

    cout << endl;
    cout << "Number of iterations: " << lastTraj.iter + 1 << endl;
    cout << "Final cost: " << lastTraj.finalCost << endl;
    cout << "Final gradient: " << lastTraj.finalGrad << endl;
    cout << "Final lambda: " << lastTraj.finalLambda << endl;
    cout << "Execution time by time step (second): " << texec/N << endl;
    cout << "Execution time per iteration (second): " << texec/lastTraj.iter << endl;
    cout << "Total execution time of the solver (second): " << texec << endl;
    cout << "\tTime of derivative (second): " << lastTraj.time_derivative.sum() << " (" << 100.0*lastTraj.time_derivative.sum()/texec << "%)" << endl;
    cout << "\tTime of backward pass (second): " << lastTraj.time_backward.sum() << " (" << 100.0*lastTraj.time_backward.sum()/texec << "%)" << endl;

    #if useUDPSolver    
    cout << "\t\tUDP backward propagation (second): " << lastTraj.time_range1.sum() << " (" << 100.0*lastTraj.time_range1.sum()/texec << "%)" << endl;
    cout << "\t\t\t20 multi-threading computation (second): " << lastTraj.time_range2.sum() << " (" << 100.0*lastTraj.time_range2.sum()/texec << "%)" << endl;
    cout << "\t\t\tMain thread computation (second): " << lastTraj.time_range3.sum() << " (" << 100.0*lastTraj.time_range3.sum()/texec << "%)" << endl;
    cout << "\tTime of forward pass (second): " << lastTraj.time_forward.sum() << " (" << 100.0*lastTraj.time_forward.sum()/texec << "%)" << endl;
    cout << "\tTotal number of a forward dynamics in main thread: " << finalTimeProfile.counter0_ << endl;
    cout << "\tTotal number of a forward dynamics in one worker thread: " << finalTimeProfile.counter1_ << endl;
    cout << "\tTotal number of a forward dynamics in one worker thread: " << finalTimeProfile.counter2_ << endl;
    // cout << "-----------" << endl;
    cout << "\tTime of one RBT block in main thread (second): " << finalTimeProfile.time_period1 << " (" << 100.0*finalTimeProfile.time_period1/texec << "%)" << endl;
    cout << "\tTime of one RBT block in one worker thread (second): " << finalTimeProfile.time_period2 << " (" << 100.0*finalTimeProfile.time_period2/texec << "%)" << endl;
    cout << "\tTime of one RBT block in one worker thread (second): " << finalTimeProfile.time_period3 << " (" << 100.0*finalTimeProfile.time_period3/texec << "%)" << endl;
    cout << "\tTime of update func (second): " << finalTimeProfile.time_period4 << " (" << 100.0*finalTimeProfile.time_period4/texec << "%)" << endl;
    #endif

    // cout << "\tTime of RBT massMatrix.inverse (second): " << finalTimeProfile.time_period2 << " (" << 100.0*finalTimeProfile.time_period2/texec << "%)" << endl;
    // cout << "\tTime of RBT f_ext instantiation (second): " << finalTimeProfile.time_period3 << " (" << 100.0*finalTimeProfile.time_period3/texec << "%)" << endl;
    // cout << "\tTime of RBT dynamicsBiasTerm (second): " << finalTimeProfile.time_period4 << " (" << 100.0*finalTimeProfile.time_period4/texec << "%)" << endl;
    
    // // debugging trajectory and control outputs (print func)
    // cout << "--------- final joint state trajectory ---------" << endl;
    // for(unsigned int i=0;i<=N;i++){
    //   cout << "lastTraj.xList[" << i << "]:" << lastTraj.xList[i].transpose() << endl;
    // }
    // cout << "--------- final joint torque trajectory ---------" << endl;
    
    // for(unsigned int i=0;i<=N;i++){
    //   cout << "lastTraj.uList[" << i << "]:" << lastTraj.uList[i].transpose() << endl;
    // }

    cout << "lastTraj.xList[" << N << "]:" << lastTraj.xList[N].transpose() << endl;
    cout << "lastTraj.uList[" << N << "]:" << lastTraj.uList[N].transpose() << endl;

    cout << "lastTraj.xList[0]:" << lastTraj.xList[0].transpose() << endl;
    cout << "lastTraj.uList[0]:" << lastTraj.uList[0].transpose() << endl;

    for(unsigned int i=0;i<=N;i++){
      cout << "lastTraj.xList[" << i << "]:" << lastTraj.xList[i].transpose() << endl;
    }


//This is my comment
/*
    // saving data file
    for(unsigned int i=0;i<N;i++){
      saveVector(joint_state_traj[i], "joint_trajectory");
      saveVector(torque_traj[i], "joint_torque_command");
    }
    saveVector(lastTraj.xList[N], "joint_trajectory");

    for(unsigned int i=0;i<=N*InterpolationScale;i++){
      saveVector(joint_state_traj_interp[i], "joint_trajectory_interpolated");
    }
*/
    cout << "-------- DDP Trajectory Generation Finished! --------" << endl;
    // traj_knot_number_ = 0;

    // Send over points using LCM
    // need this for dynamic memory allocation (push_back)
    auto ptr = std::make_unique<lcmt_ddp_traj>();
    
    ptr->dim_torques = 0;//kNumJoints;
    ptr->dim_states = kNumJoints; //disregard joint velocity
    ptr->n_time_steps = NumberofKnotPt*InterpolationScale; 
    ptr->cost = lastTraj.finalCost;

//****************************************************************************************************************
//****************************************************************************************************************
// DOES NOT MATTER FOR POSITION CONTROL -- ALSO JUST FOR DEBUGGING TORQUE, 
// EVERYTHING RELATED TO THIS SHOULD BE HANDLED IN "kuka_arm.cpp"
//****************************************************************************************************************
    // //==========================
    // // const char* kIiwaUrdf = "drake/manipulation/models/iiwa_description/urdf/iiwa7.urdf";
    // const char* const kIiwaUrdf = "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";
    // const char* schunkPath = "drake/manipulation/models/wsg_50_description/urdf/wsg_50_mesh_collision_no_world_joint.urdf";
    // const char* connectorPath = "drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf";
    
    // std::string urdf_;
    // RigidBodyTree<double> kukaTree_;
    // std::unique_ptr<RigidBodyTree<double>> totalTree_;
    // totalTree_ = std::make_unique<RigidBodyTree<double>>();

    // VectorX<double> states;
    // Eigen::VectorXd robot_q;
    // Eigen::VectorXd iiwa_q;
    // Eigen::VectorXd iiwa_v;
    // std::unique_ptr<WorldSimTreeBuilder<double>> tree_builder; 
    // std::unique_ptr<RigidBodyPlant<double>> plant;

    // robot_q = Eigen::VectorXd(9);
    // iiwa_q = Eigen::VectorXd(7);
    // iiwa_v = Eigen::VectorXd(7);

    // tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
    // tree_builder->StoreDrakeModel("iiwa", kIiwaUrdf);
    // tree_builder->StoreDrakeModel("wsg", schunkPath);
    // tree_builder->StoreDrakeModel("connector", connectorPath);


    // const Eigen::Vector3d kRobotBase(0, 0, 0);


    // ModelInstanceInfo<double> iiwa_instance, connector_instance, wsg_instance;


    // int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
    // iiwa_instance = tree_builder->get_model_info_for_instance(id);


    // id = tree_builder->AddModelInstanceToFrame(
    //     "connector", tree_builder->tree().findFrame("iiwa_frame_ee"),
    //     drake::multibody::joints::kFixed);
    // connector_instance = tree_builder->get_model_info_for_instance(id);
    

    // //The schunk will be 0.09 above the kuka end effector -> change number by going to the urdf of iiwa7
    // id = tree_builder->AddModelInstanceToFrame(
    //     "wsg", tree_builder->tree().findFrame("connector_frame"),
    //     drake::multibody::joints::kFixed);
    // wsg_instance = tree_builder->get_model_info_for_instance(id);

    // totalTree_ = tree_builder->Build();

    // VectorXd q(9);
    // VectorXd qd(9);
    //   // VectorX<double> q = VectorX<double>::Ones(7);
    //   // VectorX<double> qd = VectorX<double>::Zero(7);
    // //  q << 0,0.602162,0.00180032,-1.75906,-0.0181392,0.962099,0.0121962;
    // // q << 1,1,1,1,1,1,1,0,0;
    // q << 0, 0.6, 0, -1.75, 0, 1.0, 0, 0, 0;
    // // q << 0,0.5,0,0.5,0,0,0;
    // qd << 0,0,0,0,0,0,0, 0,0;

    // // std::unique_ptr<RigidBodyTree<double>> robot_thread_;
    // // robot_thread_ = std::make_unique<RigidBodyTree<double>>();
    // // const char* const kIiwaUrdf =
    // //         "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";
    // // parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    // //         FindResourceOrThrow(kIiwaUrdf),
    // //         multibody::joints::kFixed, robot_thread_.get());

    // KinematicsCache<double> cache_ = totalTree_->CreateKinematicsCache();
    // cache_.initialize(q,qd);
    // totalTree_->doKinematics(cache_, true);

    // MatrixX<double> M_ = totalTree_->massMatrix(cache_); // Inertial matrix

    // drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext;

    // // VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
    // VectorX<double> gtau_wb = totalTree_->inverseDynamics(cache_, f_ext, qd, false);

    // cout << "bias total" << endl << gtau_wb << endl;

    // //=======================
    // // Find Arm Gravity Compensation
    // VectorXd q_a(7);
    // VectorXd qd_a(7);
    // // q_a << 1,1,1,1,1,1,1;
    // q_a << 0, 0.6, 0, -1.75, 0, 1.0, 0;
    // qd_a << 0,0,0,0,0,0,0;

    // std::unique_ptr<RigidBodyTree<double>> robot_thread_;
    // robot_thread_ = std::make_unique<RigidBodyTree<double>>();
    // const char* const kIiwaUrdf_a =
    //         "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";
    // parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    //         FindResourceOrThrow(kIiwaUrdf_a),
    //         multibody::joints::kFixed, robot_thread_.get());

    // KinematicsCache<double> cache_a_ = robot_thread_->CreateKinematicsCache();
    // cache_a_.initialize(q_a,qd_a);
    // robot_thread_->doKinematics(cache_a_, true);

    // MatrixX<double> M_a = robot_thread_->massMatrix(cache_a_); // Inertial matrix

    // drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext_a;

    // // VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
    // VectorX<double> gtau = robot_thread_->inverseDynamics(cache_a_, f_ext_a, qd_a, false);

    // cout << "bias arm" << endl << gtau << endl;

    // // subtract biases
    // for (int k=0; k<7; k++) {
    //   gtau[k] = gtau_wb[k] - gtau[k];
    // }

    // cout << "bias subtracted" << endl << gtau << endl;

      //============================================
//****************************************************************************************************************
//****************************************************************************************************************


    for (int32_t i=0; i < ptr->n_time_steps; ++i) {
      // need new, cuz dynamic allocation or pointer
      ptr->times_sec.push_back(static_cast<double>(TimeStep*i/InterpolationScale));
      // cout << TimeStep*i/InterpolationScale << endl;
      // cout << ptr->times_sec[i] << endl;

      auto ptr2 = std::make_unique<std::vector<double>>();
      auto ptr2_st = std::make_unique<std::vector<double>>();

      for (int32_t j=0; j < ptr->dim_states; ++j) { 
        //  ptr2->push_back(lastTraj.uList[i][j]);
        // ptr2->push_back(gtau[j]);

        ptr2->push_back(0);
        ptr2_st->push_back(joint_state_traj_interp[i][j]);
      }
      ptr->torques.push_back(*ptr2);
      ptr->states.push_back(*ptr2_st);
    }

    cout << "check1" << endl;

    // need this because...?
    ddp_traj_ = *ptr;

    cout << "check2" << endl;

    // cout << ddp_traj_.dim_torques << endl;
    // cout << ddp_traj_.dim_states << endl;
    // cout << ddp_traj_.n_time_steps << endl;
    // cout << ddp_traj_.cost << endl;

    // //sanity check
//     for (int32_t i=0; i<ddp_traj_.n_time_steps; ++i) {
////       cout << ddp_traj_.times_sec[i] << endl;
//       for (int32_t j=0; j<ddp_traj_.dim_torques; ++j) {
//         cout << "torq? " << ddp_traj_.torques[i][j] << endl;
//       }
//     }

    lcm_.publish(kLcmPlanChannel, &ddp_traj_);
    cout << "-------- DDP Trajectory Published to LCM! --------" << endl;
//    for(unsigned int i=0;i<N;i++)
//    {
//      cout << "lastTraj.uList[" << i << "]:" << lastTraj.uList[i].transpose() << endl;
//    }
//
    for(unsigned int i=0;i<=N;i++){
      cout << "lastTraj.xList[" << i << "]:" << lastTraj.xList[i].transpose() << endl;
    }
  }

  void saveVector(const Eigen::MatrixXd & _vec, const char * _name){
      std::string _file_name = "~/drake/DDP/trajectory/"; //UDP_TRAJ_DIR;
      _file_name += _name;
      _file_name += ".csv";
      clean_file(_name, _file_name);

      std::ofstream save_file;
      save_file.open(_file_name, std::fstream::app);
      for (int i(0); i < _vec.rows(); ++i){
          save_file<<_vec(i,0)<< "\t";
      }
      save_file<<"\n";
      save_file.flush();
      save_file.close();
  }

  void saveValue(double _value, const char * _name){
      std::string _file_name = UDP_TRAJ_DIR;
      _file_name += _name;
      _file_name += ".csv";
      clean_file(_name, _file_name);

      std::ofstream save_file;
      save_file.open(_file_name, std::fstream::app);
      save_file<<_value <<"\n";
      save_file.flush();
  }

  void clean_file(const char * _file_name, std::string & _ret_file){
      std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
      if(gs_fileName_string.end() == iter){
          gs_fileName_string.push_back(_file_name);
          remove(_ret_file.c_str());
      }
  }

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

int do_main() {
  RobotPlanRunner runner;

  stateVec_t xinit,xgoal;
#if WHOLE_BODY
  xinit << 0, 0.6, 0, -1.75, 0, 1.0, 0, 0, 0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
  xgoal << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
#else
  xinit << 0, 0.6, 0, -1.75, 0, 1.0, 0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
  xgoal << 1.0,1.0,1.0,1.0,1.0,1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
#endif

//  xinit << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
  // xinit << 0, 0.6, 0, -1.75, 0, 1.0, 0, 0, 0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
  // xinit << 0,0.602162,0.00180032,-1.75906,-0.0181392,0.962099,0.0121962;
//  xinit << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

  // xgoal << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    // xgoal << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    // xgoal << 0,0.602162,0.00180032,-1.75906,-0.0181392,0.962099,0.0121962;
  //  xgoal << 0.5,0.5,0.5,0,0.5,0,0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
  
  runner.RunUDP(xinit, xgoal);

  return 0;
}

}  // namespace
}  // namespace yaskawa_arm
}  // namespace examples
}  // namespace drake


int main() {
  return drake::examples::yaskawa_arm::do_main();
}
