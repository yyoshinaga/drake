#include "drake/DDP/config.h"
#include "drake/DDP/yaskawa_model.h"
#include "drake/DDP/yaskawa_cost_function.h"

//Combining Coarse and Fine Physics forManipulation using Parallel-in-Time Integration

//Goal: To solve DDP's physics simulation in parallel time
//Input: u: control or Kt and kt: control parameters
//Output: X_next: state of system at next time interval

class Parareal
{
public:

//Constructor
// YaskawaModel& yaskawaDynamicModel, CostFunctionYaskawa& yaskawaCostFunction, bool fullDDP, bool QPBox
Parareal(const int& nargout, const double& dt, const stateVec_t& xList_curr, const commandVec_t& uList_curr, 
 const bool& isUNan, stateVec_t& xList_next, CostFunctionYaskawa*& costFunction);

private:

//Combining Coarse and Fine Physics for Manipulation using Parallel-in-Time Integration (Eq3)
stateVec_t doParareal();

//Coarse Predictive Physics Model: Kinematics
stateVec_t Coarse();

//Fine Predictive Physics Model: Runge-Kutta based Dynamics
stateVec_t Fine();

//Number of Parareal iterations
int k; 

//Timestep n = 0, ...,N-1
int n;

};


