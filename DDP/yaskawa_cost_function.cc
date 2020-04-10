#include "drake/DDP/yaskawa_cost_function.h"

namespace drake {
namespace examples {
namespace yaskawa_arm {
	
CostFunctionYaskawa::CostFunctionYaskawa()
{    
    // "f" means final
    pos_scale = 1;
    vel_scale = 10;
    pos_f_scale = 1000;//0.001;
    vel_f_scale = 1000;//10;
    torque_scale = 1;//100;

    // initial, final costs (pos ,vel)
    // torque cost
    // l = sigma(xQx+uRu) + xfQfxf
    #if WHOLE_BODY
        QDiagElementVec << pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100,  
                            vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10;
        QfDiagElementVec << pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0,
                            vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0;
        RDiagElementVec << torque_scale*0.005, torque_scale*0.005, torque_scale*0.007, torque_scale*0.007, torque_scale*0.02, torque_scale*0.02, torque_scale*0.05, torque_scale*0.02, torque_scale*0.05;
    #else
        QDiagElementVec << pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100,  
                            vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10;
        QfDiagElementVec << pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0,
                            vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0;
        RDiagElementVec << torque_scale*0.005, torque_scale*0.005, torque_scale*0.007, torque_scale*0.007, torque_scale*0.02, torque_scale*0.02, torque_scale*0.05;
    #endif

    Q = QDiagElementVec.asDiagonal();
    Qf = QfDiagElementVec.asDiagonal();
    R = RDiagElementVec.asDiagonal();

    // TimeHorizon = total time 
    // TimeStep = time between two timesteps
    // N = number of knot
    N = TimeHorizon/TimeStep;
    cx_new.resize(N+1);
    cu_new.resize(N+1);
    cxx_new.resize(N+1);
    cux_new.resize(N+1);
    cuu_new.resize(N+1);
}

stateMat_t& CostFunctionYaskawa::getQ(){ return Q; }

stateMat_t& CostFunctionYaskawa::getQf(){ return Qf; }

commandMat_t& CostFunctionYaskawa::getR(){ return R; }

stateVecTab_t& CostFunctionYaskawa::getcx(){ return cx_new; }

commandVecTab_t& CostFunctionYaskawa::getcu(){ return cu_new; }

stateMatTab_t& CostFunctionYaskawa::getcxx(){ return cxx_new; }

commandR_stateC_tab_t& CostFunctionYaskawa::getcux(){ return cux_new; }

commandMatTab_t& CostFunctionYaskawa::getcuu(){ return cuu_new; }

double& CostFunctionYaskawa::getc(){ return c_new; }

}  // namespace yaskawa_arm
}  // namespace examples
}  // namespace drake
