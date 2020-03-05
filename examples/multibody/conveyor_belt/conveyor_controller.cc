#include <memory>
#include <gflags/gflags.h>
#include "drake/examples/multibody/conveyor_belt/conveyor_controller.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

DEFINE_double(belt_length, 3, "The length of the belt in meters");
DEFINE_double(belt_radius, 0.075, "The radius of the belt in meters");
DEFINE_double(platform_length, 0.05, "Platform length in the sdf file");
DEFINE_int32(num_of_platforms, 7, "The number of discrete green platforms that form the entire belt");
DEFINE_double(desired_belt_velocity, 0.96, "Desired velocity of belt in 0.9652 meters/seconds which is 190ft/min");
DEFINE_double(desired_belt_rot_vel, FLAGS_desired_belt_velocity/FLAGS_belt_radius, "Desired velocity of belt in meters/seconds");
DEFINE_double(alpha_limit, M_PI-(2*std::atan2(FLAGS_belt_radius,FLAGS_platform_length/2)), "alpha while rotating");

DEFINE_double(P, 50.0, "Proportional gain");
DEFINE_double(D, 30.0, "Derivative gain");
DEFINE_double(P2, 8.0, "Proportional gain that is higher");
DEFINE_double(D2, 70.0, "Derivative gain that is higher");

DEFINE_double(strict_P, 300.0, "Proportional gain that is very high");
DEFINE_double(strict_D, 70.0, "Proportional gain that is very high");

template<typename T>
ConveyorController<T>::ConveyorController(std::vector<geometry::FrameId> _frame_ids)
:frame_ids(_frame_ids){
    
    // Input vector for # positions and # velocities.
    const int num_of_inputs = 2+2*FLAGS_num_of_platforms;
    input_port_index = this->DeclareInputPort(systems::kVectorValued, num_of_inputs).get_index();

    // Declares the states - x_pos,theta,alpha,beta,gamma,delta
    const int num_of_states = 1+FLAGS_num_of_platforms;
    this->DeclareContinuousState(num_of_states, num_of_states, 0); //# position vectors, # velocity vectors, 0 miscellaneous vectors
    this->DeclareDiscreteState(1);    // location index

    // A 3D output vector for position: x_pos, theta, alpha,gamma
    output_port_index = this->DeclareVectorOutputPort("conveyor_state", systems::BasicVector<T>(num_of_states),
                                    &ConveyorController::CopyStateOut).get_index();  //idx=0

    // Accelerations of x_pos and theta is sent to multibody plants (mbp). Reason is because mbp doesn't want velocities
    pose_output_port = this->DeclareVectorOutputPort("pose_output", systems::BasicVector<T>(num_of_states), 
                                &ConveyorController::CalcPoseOutput).get_index();    //idx=1

    this->DeclarePeriodicDiscreteUpdateEvent(0.0005, 0.0, &ConveyorController::Update);

}

template<typename T>
ConveyorController<T>::~ConveyorController() {}

template<typename T>
void ConveyorController<T>::CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const {

    // Get current state from context.
    const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector(); 

    // Write system output.
    for(int i = 0; i < 1+FLAGS_num_of_platforms; i++){
        output->SetAtIndex(i, continuous_state_vector.CopyToVector()[i]);
    }
}

template<typename T>
void ConveyorController<T>::Update(const systems::Context<T>& context,
                                    systems::DiscreteValues<T>* updates) const {

    const double location = context.get_discrete_state()[0];

    // Get the states from the plant
    const systems::BasicVector<T>* input_vector =
        this->EvalVectorInput(context, 0);
    const auto inputVector = input_vector->CopyToVector();

    const double epsilon = 0.01;

    const double x_pos = inputVector[0];
    double theta = inputVector[1];
    //Theta wrap around 0 to 2PI
    theta = fmod(theta,M_PI*2);
    if(theta < 0){ theta += M_PI*2; }

    if(x_pos < -FLAGS_belt_length/2 && ( location == 0)){
        (*updates)[0] = 1;
    }
    else if(x_pos > FLAGS_belt_length/2 && (location == 2)){
        (*updates)[0] = 3;
    }
    else{
        if((theta <= 0+epsilon || theta >= 2*M_PI-epsilon) && (location == 3)){
            (*updates)[0] = 0;
        }
        else if((theta <= M_PI+epsilon && theta >= M_PI-epsilon) && (location == 1)){
            (*updates)[0] = 2;
        }
        else{ (*updates)[0] = location; } //Keep the same location
    }

    // drake::log()->info("The belt is at location: {} with x_pos = {}", (*updates)[0],x_pos);

}

template<typename T>
void ConveyorController<T>::CalcPoseOutput(const systems::Context<T>& context ,
                                            systems::BasicVector<T>* output) const {

    // Get the states from the plant
    const systems::BasicVector<T>* input_vector =
        this->EvalVectorInput(context, 0);
    auto inputVector = input_vector->CopyToVector();

    const double location = context.get_discrete_state()[0];

    // Set positions to variables
    const double x_pos = inputVector[0];
          double theta = inputVector[1];
          double alpha = inputVector[2];  //angle between platform1 and platform2

    const double x_vel = inputVector[FLAGS_num_of_platforms+1];
    const double omega = inputVector[FLAGS_num_of_platforms+2];  
    const int x_vel_idx = 0;
    const int omega_idx = 1;

    // Conveyor belt diagram locations:
    //              0    
    //        _____________
    //   1   (_____________)    3
    //
    //              2
    
    const double buffer = 0.025; //Used to signal deceleration
    const double epsilon = 0.01;
    const double epsilon2 = 0.001;
    // int direction = 0; //Forwards

    //Theta wrap around 0 to 2PI
    theta = fmod(theta,M_PI*2);
    if(theta < 0){ theta += M_PI*2; }

    drake::log()->info("x_pos: {}\n start theta: deg {}\n alpha: {}",  x_pos,theta*180/M_PI, alpha);

    //Location 0
    if(location == 0){
        drake::log()->info("location 0");

        //Makes theta converge around 0 degrees
        if(theta > M_PI){    
            output->SetAtIndex(omega_idx, FLAGS_strict_P*(2*M_PI+epsilon-theta)+FLAGS_strict_D*(-omega)); //W_z strict
        }
        else if(theta < M_PI){
            output->SetAtIndex(omega_idx, FLAGS_strict_P*(epsilon-theta)+FLAGS_strict_D*(-omega)); //W_z strict
        }
        else{ drake::log()->info("error location 0: theta value is outside of range near 0: {}",theta); DRAKE_DEMAND(false);}

        //If close to goal, start decelerating using PD control
        if(x_pos < -FLAGS_belt_length/2+buffer){
            output->SetAtIndex(x_vel_idx, FLAGS_P*(-FLAGS_belt_length/2-buffer - x_pos)+FLAGS_D*(-x_vel)); //V_x
            drake::log()->info("decelerating");
        }
        //Otherwise accelerate until hit desired constant velocity using PD control
        else if(x_vel < -FLAGS_desired_belt_velocity-epsilon || x_vel > -FLAGS_desired_belt_velocity+epsilon){
            output->SetAtIndex(x_vel_idx, FLAGS_D2*(-FLAGS_desired_belt_velocity-x_vel)); //V_x
            drake::log()->info("accelerating");
        }
        else{ output->SetAtIndex(x_vel_idx,0); drake::log()->info("const vel {}",x_vel); }
    }
    //Location 1
    else if(location == 1){
        drake::log()->info("location 1");
        output->SetAtIndex(x_vel_idx, FLAGS_strict_P*(-FLAGS_belt_length/2+epsilon-x_pos)+FLAGS_strict_D*(-x_vel)); //V_x strict

        //If close to goal, start decelerating using PD control
        if(theta > M_PI-buffer && theta < M_PI+buffer){
            output->SetAtIndex(omega_idx, FLAGS_P*(M_PI-theta)+FLAGS_D*(-omega)); //W_z
            drake::log()->info("decelerating:\ntheta: {}",theta);
        }
        //Otherwise accelerate until hit desired constant velocity using PD control
        else if(omega > FLAGS_desired_belt_rot_vel+epsilon || omega < FLAGS_desired_belt_rot_vel-epsilon){
            output->SetAtIndex(omega_idx, FLAGS_D2*(FLAGS_desired_belt_rot_vel-omega)); //W_z
            drake::log()->info("accelerating:\n omega: {}\ndesiredOmega: {}", omega,FLAGS_desired_belt_rot_vel);
        }
        else{ output->SetAtIndex(omega_idx,0); drake::log()->info("const omega {}",omega); }
    }
    //Location 2
    else if(location == 2){
        drake::log()->info("location 2");
        output->SetAtIndex(omega_idx, FLAGS_strict_P*(M_PI-theta)+FLAGS_strict_D*(-omega)); //W_z strict

        //If close to goal, start decelerating using PD control
        if(x_pos > FLAGS_belt_length/2-buffer){
            output->SetAtIndex(x_vel_idx, FLAGS_P*(FLAGS_belt_length/2+buffer - x_pos)+FLAGS_D*(-x_vel)); //V_x
            drake::log()->info("decelerating");
        }
        //Otherwise accelerate until hit desired constant velocity using PD control
        else if(x_vel < FLAGS_desired_belt_velocity-epsilon || x_vel > FLAGS_desired_belt_velocity+epsilon){
            output->SetAtIndex(x_vel_idx, FLAGS_D2*(FLAGS_desired_belt_velocity-x_vel)); //V_x
            drake::log()->info("accelerating");
        }
        else{ output->SetAtIndex(x_vel_idx,0); drake::log()->info("const vel {}",x_vel); }
    }
    //Location 3
    else if(location == 3){
        drake::log()->info("location 3");
        output->SetAtIndex(x_vel_idx, FLAGS_strict_P*(FLAGS_belt_length/2-epsilon2-x_pos)+FLAGS_strict_D*(-x_vel)); //V_x gets pushed back

        //If close to goal, start decelerating using PD control
        if(theta > 2*M_PI-buffer ){
            output->SetAtIndex(omega_idx, 200*(2*M_PI-epsilon-theta)+FLAGS_D*(-omega)); //W_z
            drake::log()->info("decelerating");
        }
        else if(theta < M_PI-buffer){
            output->SetAtIndex(omega_idx, 200*(epsilon-theta)+FLAGS_D*(-omega)); //W_z
            drake::log()->info("decelerating");
        }
        //Otherwise accelerate until hit desired constant velocity using PD control
        else if(omega > FLAGS_desired_belt_rot_vel+epsilon || omega < FLAGS_desired_belt_rot_vel-epsilon){
            drake::log()->info("theta: {}",theta);
            output->SetAtIndex(omega_idx, FLAGS_D2*(FLAGS_desired_belt_rot_vel-omega)); //W_z
            drake::log()->info("accelerating");
        }
        else{ output->SetAtIndex(omega_idx,0); drake::log()->info("const omega {}",omega); }
    }
    else{   
        drake::log()->info("error --> x_pos is > L: {},  x_pos < -L: {} ", x_pos >= FLAGS_belt_length/2, x_pos <= -FLAGS_belt_length/2);
        drake::log()->info("error --> theta < 0: {},  theta > 2PI: {} ", theta < 0, theta > 2*M_PI);
        drake::log()->info("Location 0 condition --> {} {} {} ", x_pos < FLAGS_belt_length/2 , x_pos > -FLAGS_belt_length/2 , (theta <= 0+epsilon || theta >= 2*M_PI-epsilon)); 
        drake::log()->info("Location 1 condition --> ({} or {}) and {}", x_pos <= -FLAGS_belt_length/2, theta > buffer , theta < M_PI); 
        drake::log()->info("Location 2 condition --> {} {} {} ", x_pos < FLAGS_belt_length/2 , x_pos > -FLAGS_belt_length/2 , (theta <= M_PI+epsilon  && theta >= M_PI-epsilon )); 
        drake::log()->info("Location 3 condition --> ({} or {}) and {}", x_pos >= FLAGS_belt_length/2, theta > M_PI+buffer , theta < 2*M_PI); 
        DRAKE_DEMAND(false);
    }



    const int n = FLAGS_num_of_platforms;
    for(int i = 2; i < n+1; i++){
        double sum = 0;
        //Sum up all the angles:
        for(int j = 1; j < i; j++){
            sum+=inputVector[j];
        }
        //For alpha:
        if(location == 1 ){
            //Time to start moving alpha
            if(-inputVector[i] >= FLAGS_alpha_limit && inputVector[i] < M_PI){ 
                output->SetAtIndex(i, 500*(-FLAGS_alpha_limit-inputVector[i])+500*(-inputVector[1+n+i]));
                drake::log()->info("rotating alpha {}:\n flagAlphaDes: {}\n alpha: {}",i,-FLAGS_alpha_limit , inputVector[i]);
                // DRAKE_DEMAND(false);
            }
            else{
                output->SetAtIndex(i,5000*(-sum - inputVector[i])+200*(-inputVector[1+n+i]));
                drake::log()->info("flat alpha {}:\n alpha: {}\n theta:{}\n M_PI-theta:{} ",i, alpha, theta, M_PI-theta);
            }
        }
        else if( location == 3){
            //Time to start moving alpha
            if( inputVector[i-1] < 2*M_PI-buffer && inputVector[i-1] > M_PI && inputVector[i-1] > FLAGS_alpha_limit+M_PI){ 
                output->SetAtIndex(i, 1000*(-(M_PI-FLAGS_alpha_limit)-inputVector[i])+500*(-inputVector[1+n+i]));
            }
            else{
                output->SetAtIndex(i, 2500*((-inputVector[i-1])-inputVector[i])+200*(-inputVector[1+n+i]));
            }
        }
        else{
            //Straighten out the conveyor belt
            output->SetAtIndex(i, 1000*(-sum)+500*(-inputVector[1+n+i]));
            drake::log()->info("0 alpha {}",i);
        }
    }
}

/*
        //For alpha:
        if(location == 1 ){
            //Time to start moving alpha
            if(theta > FLAGS_theta_limit && theta < M_PI){ 
                output->SetAtIndex(beta_idx, 1000*(-(M_PI-FLAGS_alpha_desired)-alpha)+500*(-beta));
                // drake::log()->info("should start rotating alpha {} {} {}",-FLAGS_alpha_desired , alpha, FLAGS_P3*(-FLAGS_alpha_desired-alpha)+FLAGS_D3*(-beta));
                // DRAKE_DEMAND(false);
            }
            else{
                output->SetAtIndex(beta_idx, 2500*((-theta)-alpha)+500*(-beta));
                // drake::log()->info("should be flat alpha {} {} {} {}", alpha, theta, M_PI-theta,FLAGS_P*((-theta)-alpha)+FLAGS_D3*(-beta));
            }
        }
        else if( location == 3){
            //Time to start moving alpha
            if( theta < 2*M_PI-buffer && theta > M_PI && theta > FLAGS_theta_limit+M_PI){ 
                output->SetAtIndex(beta_idx, 1000*(-(M_PI-FLAGS_alpha_desired)-alpha)+500*(-beta));
            }
            else{
                output->SetAtIndex(beta_idx, 2500*((-theta)-alpha)+500*(-beta));
            }
        }
        else{
            output->SetAtIndex(beta_idx, 200*(-alpha)+FLAGS_D3*(-beta));
        }
*/





//Updates States - this is taken care in multibodyplant so there's no need to do anything here
template<typename T>
void ConveyorController<T>::DoCalcTimeDerivatives(const systems::Context<T>& ,
    systems::ContinuousState<T>* ) const {}

//Initialize states
template<typename T>
void ConveyorController<T>::SetDefaultState(const systems::Context<T>&,
                    systems::State<T>* state) const {
    DRAKE_DEMAND(state != nullptr);
    
    VectorX<T> xc0(2+2*FLAGS_num_of_platforms); 
    xc0 << 0,0,0,0,0,0,0,0,0,0,0,0;   
    state->get_mutable_continuous_state().SetFromVector(xc0);

    systems::DiscreteValues<double>& loca =  state->get_mutable_discrete_state();
    loca[0] = 0;  // start conveyor at location 0

    drake::log()->info("set default state2");
}



template class ConveyorController<double>;

}
}
}
}