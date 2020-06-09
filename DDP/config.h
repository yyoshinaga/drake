#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <numeric>
#include <sys/time.h>

#define UDP_BACKWARD_INTEGRATION_METHOD 0 // 1: 4^th-order RK, 2: simple Euler method, 3: 3^rd-order RK with FOH on u (same as dircol)
#define ENABLE_QPBOX 0
#define DISABLE_QPBOX 1
#define ENABLE_FULLDDP 0
#define DISABLE_FULLDDP 1
#define WHOLE_BODY 1

#define MULTI_THREAD 0
#if MULTI_THREAD
#include <thread>
#define NUMBER_OF_THREAD 1 //12
#endif

// #if WHOLE_BODY
// #define stateSize 18 
// #define commandSize 9 
// #define fullstatecommandSize 27 
// #else
#define stateSize 12
#define commandSize 6
#define fullstatecommandSize 18
// #endif

#define TimeHorizon 1.5
#define TimeStep 0.5 //0.005
#define NumberofKnotPt TimeHorizon/TimeStep
#define InterpolationScale 10 //0.01/1e-3
const int32_t kNumJoints = 6;
#define UDP_TRAJ_DIR "~/drake/DDP/trajectory/"
const char* const kLcmQueryResultsChannel = "TREE_SEARCH_QUERY_RESULTS";

namespace drake {
namespace examples {
namespace yaskawa_arm {
namespace {

// typedef for stateSize types
typedef Eigen::Matrix<double,stateSize,1> stateVec_t;                       // stateSize x 1
typedef Eigen::Matrix<double,1,stateSize> stateVecTrans_t;                  // 1 x stateSize
typedef Eigen::Matrix<double,stateSize,stateSize> stateMat_t;               // stateSize x stateSize
typedef Eigen::Matrix<double,stateSize,stateSize> stateTens_t[stateSize];   // stateSize x stateSize x stateSize

// typedef for commandSize types
typedef Eigen::Matrix<double,commandSize,1> commandVec_t;                           // commandSize x 1
typedef Eigen::Matrix<double,1,commandSize> commandVecTrans_t;                      // 1 x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandMat_t;                 // commandSize x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandTens_t[commandSize];   // stateSize x commandSize x commandSize

// typedef for mixed stateSize and commandSize types
typedef Eigen::Matrix<double,stateSize,commandSize> stateR_commandC_t;                          // stateSize x commandSize
typedef Eigen::Matrix<double,stateSize,commandSize> stateR_commandC_stateD_t[stateSize];        // stateSize x commandSize x stateSize
typedef Eigen::Matrix<double,stateSize,commandSize> stateR_commandC_commandD_t[commandSize];    // stateSize x commandSize x commandSize
typedef Eigen::Matrix<double,commandSize,stateSize> commandR_stateC_t;                          // commandSize x stateSize
typedef Eigen::Matrix<double,commandSize,stateSize> commandR_stateC_stateD_t[stateSize];        // commandSize x stateSize x stateSize
typedef Eigen::Matrix<double,commandSize,stateSize> commandR_stateC_commandD_t[commandSize];    // commandSize x stateSize x commandSize
typedef Eigen::Matrix<double,stateSize,stateSize> stateR_stateC_commandD_t[commandSize];        // stateSize x stateSize x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandR_commandC_stateD_t[stateSize];    // commandSize x commandSize x stateSize
typedef Eigen::Matrix<double,stateSize+commandSize,1> stateAug_t;                               // stateSize + commandSize x 1
typedef Eigen::Matrix<double,1,1> scalar_t;                                                     // 1 x 1

// typedef for half commandSize and stateSize types
typedef Eigen::Matrix<double,stateSize/2,1> stateVec_half_t;                                    // stateSize/2 x 1
typedef Eigen::Matrix<double,stateSize/2,stateSize/2> stateMat_half_t;                          // stateSize/2 x stateSize/2
typedef Eigen::Matrix<double,stateSize/2,commandSize> stateR_half_commandC_t;                   // stateSize/2 x commandSize

// typedef for vectorized state and command matrix (over the horizon)
typedef std::vector<stateVec_t> stateVecTab_t;
typedef std::vector<double> costVecTab_t;
typedef std::vector<commandVec_t> commandVecTab_t;
typedef std::vector<stateMat_t> stateMatTab_t;
typedef std::vector<commandMat_t> commandMatTab_t;
typedef std::vector<stateR_commandC_t> stateR_commandC_tab_t;
typedef std::vector<commandR_stateC_t> commandR_stateC_tab_t;
typedef std::vector<stateVec_half_t> stateVecTab_half_t;

//typedef std::vector<stateTens_t> stateTensTab_t;
typedef std::vector<std::vector<stateMat_t> > stateTensTab_t;
typedef std::vector<std::vector<stateR_commandC_t> > stateR_commandC_Tens_t;

}  // namespace
}  // namespace yaskawa_arm
}  // namespace examples
}  // namespace drake

#endif // CONFIG_H