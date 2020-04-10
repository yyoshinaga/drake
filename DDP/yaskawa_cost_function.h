#pragma once

#ifndef COSTFUNCTIONYASKAWA_H
#define COSTFUNCTIONYASKAWA_H

#include "drake/DDP/config.h"
#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;

namespace drake {
namespace examples {
namespace yaskawa_arm {

class CostFunctionYaskawa
{
public:
    CostFunctionYaskawa();
private:
protected:
	stateMat_t Q;
	stateMat_t Qf;
	commandMat_t R;

	stateVec_t QDiagElementVec;
	stateVec_t QfDiagElementVec;
	commandVec_t RDiagElementVec;
	double pos_scale;
    double vel_scale;
    double pos_f_scale;
    double vel_f_scale;
    double torque_scale;
    
	stateVecTab_t cx_new;
	commandVecTab_t cu_new; 
	stateMatTab_t cxx_new; 
	commandR_stateC_tab_t cux_new; 
	commandMatTab_t cuu_new;
	double c_new;
    // attributes
public:
	stateMat_t& getQ();
	stateMat_t& getQf();
	commandMat_t& getR();
	stateVecTab_t& getcx();
	commandVecTab_t& getcu();
	stateMatTab_t& getcxx();
	commandR_stateC_tab_t& getcux();
	commandMatTab_t& getcuu();
	double& getc();

	unsigned int N;
private:

protected:
    // methods
public:
private:
protected:
    // accessors
public:

};

}  // namespace yaskawa_arm
}  // namespace examples
}  // namespace drake

#endif // COSTFUNCTIONYASKAWA_H
