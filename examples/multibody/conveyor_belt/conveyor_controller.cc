#include <memory>
#include <gflags/gflags.h>
#include "drake/examples/multibody/conveyor_belt/conveyor_controller.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

DEFINE_double(belt_length, 3, "The length of the belt in meters");
DEFINE_double(belt_radius, 0.5, "The radius of the belt in meters");
DEFINE_double(platform_length, 0.15, "Platform length in the sdf file");
DEFINE_double(theta_limit, 2*std::atan2((FLAGS_platform_length/2),FLAGS_belt_radius), "when alpha starts to increase in value");

DEFINE_double(x_pos_start, FLAGS_belt_length/2, "The x position of the platform in meters");
DEFINE_double(theta_start, 0, "The angle of the platform in rad");
DEFINE_double(desired_belt_velocity, 0.4, "Desired velocity of belt in meters/seconds");
DEFINE_double(desired_belt_rot_vel, FLAGS_desired_belt_velocity/FLAGS_belt_radius, "Desired velocity of belt in meters/seconds");
DEFINE_double(alpha_desired, std::atan2(FLAGS_belt_radius,FLAGS_platform_length), "alpha while rotating");

DEFINE_double(P, 50.0, "Proportional gain");
DEFINE_double(D, 30.0, "Derivative gain");
DEFINE_double(P2, 8.0, "Proportional gain that is higher");
DEFINE_double(D2, 70.0, "Derivative gain that is higher");
DEFINE_double(P3, 10.0, "Proportional gain that is higher");
DEFINE_double(D3, 50.0, "Derivative gain that is higher");

template<typename T>
ConveyorController<T>::ConveyorController(std::vector<geometry::FrameId> _frame_ids)
:frame_ids(_frame_ids){
    
    // A 6D input vector for 3 positions and 3 velocities.
    input_port_index = this->DeclareInputPort(systems::kVectorValued, 6).get_index();

    // Declares the states
    this->DeclareContinuousState(3, 3, 0); //3 position vectors, 3 velocity vectors, 1 miscellaneous vectors
    this->DeclareDiscreteState(1);    // location

    // A 3D output vector for position: x_pos, theta, gamma
    output_port_index = this->DeclareVectorOutputPort("conveyor_state", systems::BasicVector<T>(3),
                                    &ConveyorController::CopyStateOut).get_index();  //0

    // Accelerations of x_pos and theta is sent to multibody plants (mbp). Reason is because mbp doesn't want velocities
    pose_output_port = this->DeclareVectorOutputPort("pose_output", systems::BasicVector<T>(3), 
                                &ConveyorController::CalcPoseOutput).get_index();    //1

    this->DeclarePeriodicDiscreteUpdateEvent(1/50, 0.0, &ConveyorController::Update);

    this->DeclareVectorOutputPort("Sn", systems::BasicVector<double>(1),
                                &ConveyorController::Output);
}

template<typename T>
ConveyorController<T>::~ConveyorController() {}

template<typename T>
void ConveyorController<T>::CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const {

    // Get current state from context.
    const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector(); 

    // Write system output.
    output->SetAtIndex(0, continuous_state_vector.CopyToVector()[0]);
    output->SetAtIndex(1, continuous_state_vector.CopyToVector()[1]);
    output->SetAtIndex(2, continuous_state_vector.CopyToVector()[2]);
}

template<typename T>
void ConveyorController<T>::Output(const systems::Context<T>& context,
            systems::BasicVector<T>* result) const {
(*result)[0] = 0;
}

template<typename T>
void ConveyorController<T>::Update(const systems::Context<T>& context,
                                    systems::DiscreteValues<T>* updates) const {
    drake::log()->info("THIS MESSAGE SHOULD APPEAR");

    const double location = context.get_discrete_state()[0];

    const systems::VectorBase<T>& inputVector =
      context.get_continuous_state_vector(); 
    drake::log()->info("discrete location: {} {} {}", inputVector[0], inputVector[1], inputVector[2]);

      drake::log()->info("intputvector size: {}",(*updates).size());

    const double epsilon = 0.01;
    if(inputVector[0] < -FLAGS_belt_length/2 && (location == 1 || location == 0)){
        (*updates)[0] = 1;
    }
    else if(inputVector[0] > FLAGS_belt_length/2 && (location == 3 || location == 2)){
        (*updates)[0] = 3;
    }
    else{
        if((inputVector[1] <= 0+epsilon || inputVector[1] >= 2*M_PI-epsilon) && (location == 0 || location == 3)){
            (*updates)[0] = 0;
        }
        else if((inputVector[1] <= M_PI+epsilon && inputVector[1] >= M_PI-epsilon) && (location == 2 || location == 1)){
            (*updates)[0] = 2;
        }
        else{drake::log()->info("failed to find correct location"); DRAKE_DEMAND(false);}
    }
}

template<typename T>
void ConveyorController<T>::CalcPoseOutput(const systems::Context<T>& context ,
                                            systems::BasicVector<T>* output) const {

    // Get the states from the plant
    const systems::BasicVector<T>* input_vector =
        this->EvalVectorInput(context, 0);
    auto inputVector = input_vector->CopyToVector();

    const double location = context.get_discrete_state()[0];
    drake::log()->info("GEtting location from contini: {} {}  ", location, inputVector[1]);


    // Set positions to variables
    const double x_pos = inputVector[0];
          double theta = inputVector[1];
    const double alpha = inputVector[2];  //angle between platform1 and platform2

    const double x_vel = inputVector[3];  //Make sure to change this to 2 if there exists only one conveyor belt
    const double omega = inputVector[4];  //Make sure to change this to 3 if there exists only one conveyor belt
    const double beta  = inputVector[5];  //Angular velocity for alpha
    const int x_vel_idx = 0;
    const int omega_idx = 1;
    const int beta_idx = 2;

    // Conveyor belt diagram locations:
    //              0    
    //        _____________
    //   1   (_____________)    3
    //
    //              2
    
    const double buffer = 0.025; //Used to signal deceleration
    const double epsilon = 0.01;
    // int direction = 0; //Forwards

    //Theta wrap around 0 to 2PI
    theta = fmod(theta,M_PI*2);
    if(theta < 0){ theta += M_PI*2; }

    drake::log()->info("start theta: deg {}  x_pos: {} alpha: {}", theta*180/M_PI, x_pos, alpha);

    //Location 0
    if(x_pos < FLAGS_belt_length/2 && x_pos > -FLAGS_belt_length/2 && (theta <= 0+epsilon || theta >= 2*M_PI-epsilon)){
        drake::log()->info("location 0");

        //Makes theta converge around 0 degrees
        if(theta > 2*M_PI-buffer){    
            output->SetAtIndex(omega_idx, FLAGS_P*(2*M_PI+epsilon-theta)+FLAGS_D*(-omega)); //W_z strict
            drake::log()->info("theta neg");
        }
        else if(theta < 0+buffer){
            output->SetAtIndex(omega_idx, FLAGS_P*(-theta)+FLAGS_D*(-omega)); //W_z strict
            drake::log()->info("theta plus");
        }
        else{ drake::log()->info("error location 0: theta value is outside of range near 0"); /*DRAKE_DEMAND(false);*/}

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
    else if((x_pos <= -FLAGS_belt_length/2 || theta > 0+epsilon) && theta < M_PI){
        drake::log()->info("location 1");
        output->SetAtIndex(x_vel_idx, FLAGS_P*(-FLAGS_belt_length/2+epsilon-x_pos)+FLAGS_D*(-x_vel)); //V_x strict

        //If close to goal, start decelerating using PD control
        if(theta > M_PI-buffer){
            output->SetAtIndex(omega_idx, FLAGS_P*(M_PI+buffer-theta)+FLAGS_D*(-omega)); //W_z
            drake::log()->info("decelerating");
        }
        //Otherwise accelerate until hit desired constant velocity using PD control
        else if(omega > FLAGS_desired_belt_rot_vel+epsilon || omega < FLAGS_desired_belt_rot_vel-epsilon){
            output->SetAtIndex(omega_idx, FLAGS_D2*(FLAGS_desired_belt_rot_vel-omega)); //W_z

            drake::log()->info("accelerating");
        }
        else{ output->SetAtIndex(omega_idx,0); drake::log()->info("const omega {}",omega); }
    }
    //Location 2
    else if(x_pos < FLAGS_belt_length/2 && x_pos > -FLAGS_belt_length/2 && (theta <= M_PI+epsilon && theta >= M_PI-epsilon )){
        drake::log()->info("location 2");
        output->SetAtIndex(omega_idx, 1000000*(M_PI-theta)+FLAGS_D*(-omega)); //W_z strict

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
    else if((x_pos >= FLAGS_belt_length/2 || theta > M_PI+buffer) && theta < 2*M_PI){
        drake::log()->info("location 3");
        output->SetAtIndex(x_vel_idx, 10*(FLAGS_belt_length/2-epsilon-x_pos)+30*(-x_vel)); //V_x gets pushed back

        //If close to goal, start decelerating using PD control
        if(theta > 2*M_PI-buffer){
            output->SetAtIndex(omega_idx, FLAGS_P*(2*M_PI+buffer-theta)+FLAGS_D*(-omega)); //W_z
            drake::log()->info("decelerating");
        }
        //Otherwise accelerate until hit desired constant velocity using PD control
        else if(omega > FLAGS_desired_belt_rot_vel+epsilon || omega < FLAGS_desired_belt_rot_vel-epsilon){
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

    //For alpha:
    if(x_pos < -FLAGS_belt_length/2 || x_pos > FLAGS_belt_radius){
        //Time to start moving alpha
        if(theta > FLAGS_theta_limit || (theta > M_PI && theta > FLAGS_theta_limit+M_PI)){ 
 
            output->SetAtIndex(beta_idx, FLAGS_P3*(-FLAGS_alpha_desired-alpha)+FLAGS_D3*(-beta));
            drake::log()->info("should start rotating alpha {}", FLAGS_P3*(FLAGS_alpha_desired-alpha)+FLAGS_D3*(-beta));
        }
        else{

            output->SetAtIndex(beta_idx, FLAGS_P3*((M_PI-theta)-alpha)+FLAGS_D3*(-beta));
            drake::log()->info("should be flat alpha {} ", FLAGS_P3*((M_PI-theta)-alpha)+FLAGS_D3*(-beta));
        }
    }
    else{
        output->SetAtIndex(beta_idx, FLAGS_P3*(-alpha)+FLAGS_D3*(-beta));
        drake::log()->info("last {} ",FLAGS_P3*(-alpha)+FLAGS_D3*(-beta));
    }

    // output->SetAtIndex(beta_idx, 0);
    // output->SetAtIndex(beta_idx, 0);
            // output->SetAtIndex(2, 30*(-alpha)+15*(-beta)); //W_z strict


    // output->SetAtIndex(0,-1); //acc
    // output->SetAtIndex(1,0);
    // output->SetAtIndex(2,1);
    // drake::log()->info("input1: {}\n intput2: {}\n intput3: {}\n intput4: {}", inputVector[0], inputVector[1], inputVector[2], inputVector[3]); //inputVector[4], inputVector[5]
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
    
    VectorX<T> xc0(6);    
    xc0 << FLAGS_theta_start, FLAGS_x_pos_start, 0, 0, 0, 0;  // initial state values.
    
    state->get_mutable_continuous_state().SetFromVector(xc0);

    systems::DiscreteValues<double>& xd =  state->get_mutable_discrete_state();
    xd[0] = 0;  // xd_0
    drake::log()->info("set default state2");
}



template class ConveyorController<double>;

}
}
}
}