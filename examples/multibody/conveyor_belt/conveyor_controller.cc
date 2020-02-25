#include <memory>
#include <gflags/gflags.h>
#include "drake/examples/multibody/conveyor_belt/conveyor_controller.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

DEFINE_double(belt_length, 3.0, "The length of the belt in meters");
DEFINE_double(belt_radius, 1.0, "The radius of the belt in meters");

DEFINE_double(x_pos_start, FLAGS_belt_length/2, "The x position of the platform in meters");
DEFINE_double(theta_start, 0, "The angle of the platform in rad");
DEFINE_double(desired_belt_velocity, 0.5, "Desired velocity of belt in meters/seconds");
DEFINE_double(desired_belt_rot_vel, FLAGS_desired_belt_velocity/FLAGS_belt_radius, "Desired velocity of belt in meters/seconds");

DEFINE_double(P, 8.0, "Proportional gain");
DEFINE_double(D, 15.0, "Derivative gain");
DEFINE_double(P2, 8.0, "Proportional gain that is higher");
DEFINE_double(D2, 30.0, "Derivative gain that is higher");


template<typename T>
ConveyorController<T>::ConveyorController(std::vector<geometry::FrameId> _frame_ids)
:frame_ids(_frame_ids){
    
    // A 5D input vector for 3 positions and 3 velocities.
    input_port_index = this->DeclareInputPort(systems::kVectorValued, 4).get_index();

    // Declares the states
    this->DeclareContinuousState(2, 2, 0); //3 position vectors, 3 velocity vectors, 0 miscellaneous vectors

    // A 3D output vector for position: x_pos, theta, gamma
    output_port_index = this->DeclareVectorOutputPort("conveyor_state", systems::BasicVector<T>(2),
                                    &ConveyorController::CopyStateOut).get_index();  //0

    // Accelerations of x_pos and theta is sent to multibody plants (mbp). Reason is because mbp doesn't want velocities
    pose_output_port = this->DeclareVectorOutputPort("pose_output", systems::BasicVector<T>(2), 
                                &ConveyorController::CalcPoseOutput).get_index();    //1
}

template<typename T>
ConveyorController<T>::~ConveyorController() {}

template<typename T>
void ConveyorController<T>::CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const {

    // Get current state from context.
    const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector(); 

    drake::log()->info("4 cont st vec: {}", continuous_state_vector[3]);
    // drake::log()->info("5 cont st vec: {}", continuous_state_vector[4]);
    // drake::log()->info("6 cont st vec: {}", continuous_state_vector[5]);
    drake::log()->info("size: {}", continuous_state_vector.size());


    // Write system output.
    output->SetAtIndex(0, continuous_state_vector.CopyToVector()[0]);
    output->SetAtIndex(1, continuous_state_vector.CopyToVector()[1]);
    // output->SetAtIndex(2, continuous_state_vector.CopyToVector()[2]);

}

template<typename T>
void ConveyorController<T>::CalcPoseOutput(const systems::Context<T>& context ,
                                            systems::BasicVector<T>* output) const {

    // Get the states from the plant
    const systems::BasicVector<T>* input_vector =
        this->EvalVectorInput(context, 0);
    auto inputVector = input_vector->CopyToVector();

    // // Set positions to variables
    // double x_pos = inputVector[0];
    // double theta = inputVector[1];
    // // double alpha = inputVector[2];  //angle between platform1 and platform2

    // double x_vel = inputVector[3];
    // double omega = inputVector[4];
    // double omega_alpha = inputVector[5];

    // // Conveyor belt diagram locations:
    // //              0    
    // //        _____________
    // //   1   (_____________)    3
    // //
    // //              2
    
    // double buffer = 0.025; //Used to signal deceleration
    // double epsilon = 0.01;
    // // int direction = 0; //Forwards

    // //Theta wrap around 0 to 2PI
    // theta = fmod(theta,M_PI*2);
    // if(theta < 0){ theta += M_PI*2; }

    // drake::log()->info("start theta: deg {}  x_pos: {}", theta*180/M_PI, x_pos);

    // //Location 0
    // if(x_pos < FLAGS_belt_length/2 && x_pos > -FLAGS_belt_length/2 && (theta <= 0+epsilon || theta >= 2*M_PI-epsilon)){
    //     drake::log()->info("location 0");

    //     //Makes theta converge around 0 degrees
    //     if(theta > 2*M_PI-buffer){    
    //         output->SetAtIndex(1, 10*(2*M_PI+epsilon-theta)+30*(-omega)); //W_z strict
    //     }
    //     else if(theta < 0+buffer){
    //         output->SetAtIndex(1, 10*(-theta)+30*(-omega)); //W_z strict
    //     }
    //     else{ drake::log()->info("error location 0: theta value is outside of range near 0"); DRAKE_DEMAND(false);}

    //     //If close to goal, start decelerating using PD control
    //     if(x_pos < -FLAGS_belt_length/2+buffer){
    //         output->SetAtIndex(0, FLAGS_P*(-FLAGS_belt_length/2-buffer - x_pos)+FLAGS_D*(-x_vel)); //V_x
    //         drake::log()->info("decelerating");
    //     }
    //     //Otherwise accelerate until hit desired constant velocity using PD control
    //     else if(x_vel < -FLAGS_desired_belt_velocity-epsilon || x_vel > -FLAGS_desired_belt_velocity+epsilon){
    //         output->SetAtIndex(0, FLAGS_D2*(-FLAGS_desired_belt_velocity-x_vel)); //V_x
    //         drake::log()->info("accelerating");
    //     }
    //     else{ output->SetAtIndex(0,0); drake::log()->info("const vel {}",x_vel); }
    // }
    // //Location 1
    // else if((x_pos <= -FLAGS_belt_length/2 && theta < M_PI) || (theta > 0+buffer && theta < M_PI)){
    //     drake::log()->info("location 1");
    //     output->SetAtIndex(0, 10*(-FLAGS_belt_length/2+epsilon-x_pos)+30*(-x_vel)); //V_x strict

    //     //If close to goal, start decelerating using PD control
    //     if(theta > M_PI-buffer){
    //         output->SetAtIndex(1, FLAGS_P*(M_PI+buffer-theta)+FLAGS_D*(-omega)); //W_z
    //         drake::log()->info("decelerating");
    //     }
    //     //Otherwise accelerate until hit desired constant velocity using PD control
    //     else if(omega > FLAGS_desired_belt_rot_vel+epsilon || omega < FLAGS_desired_belt_rot_vel-epsilon){
    //         output->SetAtIndex(1, FLAGS_D2*(FLAGS_desired_belt_rot_vel-omega)); //W_z
    //         drake::log()->info("accelerating");
    //     }
    //     else{ output->SetAtIndex(1,0); drake::log()->info("const omega {}",omega); }
    // }
    // //Location 2
    // else if(x_pos < FLAGS_belt_length/2 && x_pos > -FLAGS_belt_length/2 && (theta <= M_PI+epsilon  && theta >= M_PI-epsilon )){
    //     drake::log()->info("location 2");
    //     output->SetAtIndex(1, 10*(M_PI-theta)+30*(-omega)); //W_z strict

    //     //If close to goal, start decelerating using PD control
    //     if(x_pos > FLAGS_belt_length/2-buffer){
    //         output->SetAtIndex(0, FLAGS_P*(FLAGS_belt_length/2+buffer - x_pos)+FLAGS_D*(-x_vel)); //V_x
    //         drake::log()->info("decelerating");
    //     }
    //     //Otherwise accelerate until hit desired constant velocity using PD control
    //     else if(x_vel < FLAGS_desired_belt_velocity-epsilon || x_vel > FLAGS_desired_belt_velocity+epsilon){
    //         output->SetAtIndex(0, FLAGS_D2*(FLAGS_desired_belt_velocity-x_vel)); //V_x
    //         drake::log()->info("accelerating");
    //     }
    //     else{ output->SetAtIndex(0,0); drake::log()->info("const vel {}",x_vel); }
    // }
    // //Location 3
    // else if((x_pos >= FLAGS_belt_length/2 && theta < 2*M_PI) || (theta > M_PI+buffer && theta < 2*M_PI)){
    //     drake::log()->info("location 3");
    //     output->SetAtIndex(0, 10*(FLAGS_belt_length/2-epsilon-x_pos)+30*(-x_vel)); //V_x gets pushed back

    //     //If close to goal, start decelerating using PD control
    //     if(theta > 2*M_PI-buffer){
    //         output->SetAtIndex(1, FLAGS_P*(2*M_PI+buffer-theta)+FLAGS_D*(-omega)); //W_z
    //         drake::log()->info("decelerating");
    //     }
    //     //Otherwise accelerate until hit desired constant velocity using PD control
    //     else if(omega > FLAGS_desired_belt_rot_vel+epsilon || omega < FLAGS_desired_belt_rot_vel-epsilon){
    //         output->SetAtIndex(1, FLAGS_D2*(FLAGS_desired_belt_rot_vel-omega)); //W_z
    //         drake::log()->info("accelerating");
    //     }
    //     else{ output->SetAtIndex(1,0); drake::log()->info("const omega {}",omega); }

    // }
    // else{   
    //     drake::log()->info("error --> x_pos is > L: {},  x_pos < -L: {} ", x_pos >= FLAGS_belt_length/2, x_pos <= -FLAGS_belt_length/2);
    //     drake::log()->info("error --> theta < 0: {},  theta > 2PI: {} ", theta < 0, theta > 2*M_PI);
    //     drake::log()->info("Location 0 condition --> {} {} {} ", x_pos < FLAGS_belt_length/2 , x_pos > -FLAGS_belt_length/2 , (theta <= 0+epsilon || theta >= 2*M_PI-epsilon)); 
    //     drake::log()->info("Location 1 condition --> {} {} or {} {}", x_pos <= -FLAGS_belt_length/2 , theta < M_PI, theta > buffer , theta < M_PI); 
    //     drake::log()->info("Location 2 condition --> {} {} {} ", x_pos < FLAGS_belt_length/2 , x_pos > -FLAGS_belt_length/2 , (theta <= M_PI+epsilon  && theta >= M_PI-epsilon )); 
    //     drake::log()->info("Location 3 condition --> {} {} or {} {}", x_pos >= FLAGS_belt_length/2 , theta < 2*M_PI, theta > M_PI+buffer , theta < 2*M_PI); 
    //     // DRAKE_DEMAND(false);
    // }
    output->SetAtIndex(0,0); //acc
    output->SetAtIndex(1,0);
    // output->SetAtIndex(2,1);
    drake::log()->info("input1: {}\n intput2: {}\n intput3: {}\n intput4: {}", inputVector[0], inputVector[1], inputVector[2], inputVector[3]); //inputVector[4], inputVector[5]
}

//Updates States - this is taken care in multibodyplant so there's no need to do anything here
template<typename T>
void ConveyorController<T>::DoCalcTimeDerivatives(const systems::Context<T>& ,
    systems::ContinuousState<T>* ) const {}

//Initialize states
template<typename T>
void ConveyorController<T>::SetDefaultState(const systems::Context<T>&,
                    systems::State<T>* state) const {
    DRAKE_DEMAND(state != nullptr);
    
    VectorX<T> xc0(4);    
    xc0 << FLAGS_theta_start, FLAGS_x_pos_start, 0, 0;  // initial state values.
    
    state->get_mutable_continuous_state().SetFromVector(xc0);
    drake::log()->info("set default state2");
}



template class ConveyorController<double>;

}
}
}
}