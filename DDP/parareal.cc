#include "drake/DDP/parareal.h"

/* Debug */
#include <iostream>
using namespace std;
/* */

using namespace Eigen;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace yaskawa_arm {
{

Parareal::Parareal(const int& nargout, const double& dt, const stateVec_t& xList_curr, 
  const commandVec_t& uList_curr,  const bool& isUNan, stateVec_t& xList_next, CostFunctionYaskawa*& costFunction ){

  
}

stateVec_t Parareal::DoParareal(){
  //xᵏ⁺¹ₙ₊₁ = C(xᵏ⁺¹ₙ, uₙ, dt) + F(xᵏₙ, uₙ, dt) - C(xᵏₙ, uₙ, dt)
  return Coarse(xList_curr, uList_curr, dt, k+1, n) + Fine(xList_curr, uList_curr, dt, k,n ) - Coarse(xList_curr, k, uList_curr, dt);
}

stateVec_t Parareal::Coarse(){



}

stateVec_t Parareal::Fine(const int& nargout, const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B){
  //Do Runge-Kutta in update(). This function is inside YaskawaModel
  return YaskawaModel::update(nargout_update1, xList_curr, uList_curr, AA, BB);
}




}
} //yaskawa_arm
} //examples
} //drake
























