#include <memory>
#include <gflags/gflags.h>
#include "drake/examples/multibody/conveyor_belt/conveyor_controller2.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

DEFINE_double(belt_length, 3, "The length of the belt in meters");
DEFINE_double(belt_radius, 0.075, "0.075The radius of the belt in meters");
DEFINE_double(pusher_height,0.1, "height of the pusher");
DEFINE_double(acc_x_time, 0.1, "time to get from 0 vel to max vel");
DEFINE_double(acc_y_time, 0.02, "time to get from 0 vel to max vel");

DEFINE_double(desired_belt_velocity, 0.2, "Desired velocity of belt in 0.9652 meters/seconds which is 190ft/min");

DEFINE_double(P, 1000.0, "Proportional gain");
DEFINE_double(D, 200.0, "Derivative gain");
DEFINE_double(P2, 1000.0, "Proportional gain");
DEFINE_double(D2, 200.0, "Derivative gain");

template<typename T>
ConveyorController2<T>::ConveyorController2(std::vector<geometry::FrameId> _frame_ids)
:frame_ids(_frame_ids){
    
    // Input vector for # positions and # velocities.
    state_input_port_index = this->DeclareInputPort(systems::kVectorValued, 4).get_index();

    // Declares the states - x_pos,theta,alpha,beta,gamma,delta
    this->DeclareContinuousState(2, 2, 0); //# position vectors, # velocity vectors, 0 miscellaneous vectors
    this->DeclareDiscreteState(2);    // location index

    // A 2D output vector for position: x_pos, y_pos
    position_output_port_index = this->DeclareVectorOutputPort("conveyor_positions", systems::BasicVector<T>(2),
                                    &ConveyorController2::CopyPositionOut).get_index();  //idx=0

    // Accelerations of x_pos and theta is sent to multibody plants (mbp). Reason is because mbp doesn't want velocities
    acc_output_port_index = this->DeclareVectorOutputPort("pose_output", systems::BasicVector<T>(2), 
                                &ConveyorController2::CalcAccOutput).get_index();    //idx=1

    this->DeclarePeriodicDiscreteUpdateEvent(0.0025, 0.0, &ConveyorController2::Update);
}

template<typename T>
ConveyorController2<T>::~ConveyorController2() {}

template<typename T>
void ConveyorController2<T>::CopyPositionOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const {

    // Get current state from context.
    const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector(); 

    // Write system output.
    for(int i = 0; i < 2; i++){
        output->SetAtIndex(i, continuous_state_vector.CopyToVector()[i]);
    }
}

template<typename T>
void ConveyorController2<T>::Update(const systems::Context<T>& context,
                                    systems::DiscreteValues<T>* updates) const {

    const double location = context.get_discrete_state()[0];
    const double init_time = context.get_discrete_state()[1];

    // drake::log()->info("Update: init Time: {}", init_time);

    // Get continuous states from the plant
    const systems::BasicVector<T>* input_vector =
        this->EvalVectorInput(context, 0);
    const auto inputVector = input_vector->CopyToVector();

    const double x_pos = inputVector[0];
    const double y_pos = inputVector[1];

    if(x_pos < -FLAGS_belt_length/2 && location == 0){
        (*updates)[0] = 1;
        (*updates)[1] = context.get_time();
    }
    else if(x_pos > FLAGS_belt_length/2 && location == 2){
        (*updates)[0] = 3;
        (*updates)[1] = context.get_time();
    }
    else{
        if(y_pos > FLAGS_belt_radius && location == 3){
            (*updates)[0] = 0;
            (*updates)[1] = context.get_time();
        }
        else if(y_pos < -FLAGS_belt_radius && location == 1){
            (*updates)[0] = 2;
            (*updates)[1] = context.get_time();
        }
        else{ 
            (*updates)[0] = location;
            (*updates)[1] = init_time;
        } //Keep the same location and time
    }
    // drake::log()->info("The belt is at location: {} with x_pos = {}", (*updates)[0],x_pos);
}

template<typename T>
void ConveyorController2<T>::CalcAccOutput(const systems::Context<T>& context ,
                                            systems::BasicVector<T>* output) const {

    // Get the states from the plant
    const systems::BasicVector<T>* input_vector =
        this->EvalVectorInput(context, 0);
    auto inputVector = input_vector->CopyToVector();

    const double location = context.get_discrete_state()[0];
    const double init_time = context.get_discrete_state()[1];
    // drake::log()->info("CalcAccOutput init Time: {}", init_time);

    // Set positions to variables
    const double x_pos = inputVector[0];
    const double y_pos = inputVector[1];

    const double x_vel = inputVector[2];
    const double y_vel = inputVector[3];  
    const int x_idx = 0;
    const int y_idx = 1;

    // Conveyor belt diagram locations:
    //              0    
    //        _____________
    //   1   (_____________)    3
    //
    //              2
    
    //These should be declared as global consts or GFLAGS
    const double t = context.get_time();

    // bool forwards = true; //Direction of belt

    //Location 0
    if(location == 0){
        // drake::log()->info("location 0");
        const double to1 = init_time;
        const double tf1 = FLAGS_acc_x_time+init_time;    
        const double dxo1 = 0;
        const double dxf1 = -FLAGS_desired_belt_velocity;
        const double xo1 = FLAGS_belt_length/2;
        const double xf1 =(tf1-to1)/2*(dxf1-dxo1)+xo1;
        
        const double to2 = FLAGS_belt_length/FLAGS_desired_belt_velocity+FLAGS_acc_x_time+init_time;
        const double tf2 = FLAGS_belt_length/FLAGS_desired_belt_velocity+init_time+FLAGS_acc_x_time*2;
        const double dxo2 = -FLAGS_desired_belt_velocity;
        const double dxf2 = 0;
        const double xf2 = -FLAGS_belt_length/2;
        const double xo2 = (to2-tf2)/2*(dxo2-dxf2)+xf2;

        //drake::log()->info("to1: {}\ntf1: {}\nto2: {}\ntf2: {}\nt: {}",to1,tf1,to2,tf2,t);
        DRAKE_DEMAND(to1 >= 0 && tf1 >= 0 && to2 >= 0 && tf2 >= 0);

        //Creating the polynomial curve for x_pos
        double x_pos_des = 0;
        if(t < tf1){
            x_pos_des=FourPolyTraj(t,xo1,xf1,to1,tf1,dxo1,dxf1);
            output->SetAtIndex(x_idx, FLAGS_P*(x_pos_des-x_pos)+FLAGS_D*(-FLAGS_desired_belt_velocity-x_vel)); //PD controller
            //drake::log()->info("acceleration");
        }
        else if(t >= tf1 && t <= to2){
            x_pos_des=-FLAGS_desired_belt_velocity*(t-tf1)+xf1;
            output->SetAtIndex(x_idx, FLAGS_P*(x_pos_des-x_pos)+FLAGS_D*(-FLAGS_desired_belt_velocity-x_vel)); //PD controller
            //drake::log()->info("const vel");
        }
        else if(t > to2){
            x_pos_des=FourPolyTraj(t,xo2,xf2,to2,tf2,dxo2,dxf2);
            output->SetAtIndex(x_idx, FLAGS_P*(x_pos_des-x_pos)+FLAGS_D*(-x_vel)); //PD controller
            //drake::log()->info("deceleration");
        }
        else{ DRAKE_DEMAND(false); }
        //drake::log()->info("x_pos: {}\nx_pos_des: {}\nx_vel: {}\nPD: {}",x_pos,x_pos_des, x_vel,FLAGS_P*(x_pos_des-x_pos)+FLAGS_D*(-x_vel));

        output->SetAtIndex(y_idx, FLAGS_P2*(FLAGS_belt_radius/2-y_pos)+FLAGS_D2*(-y_vel));

    }
    //Location 1
    else if(location == 1){
        //drake::log()->info("location 1");

        // const double to1 = init_time;
        // const double tf1 = FLAGS_acc_y_time+init_time;    
        // const double dyo1 = 0;
        // const double dyf1 = -FLAGS_desired_belt_velocity;
        // const double yo1 = FLAGS_belt_radius/2+FLAGS_pusher_height;
        // const double yf1 =(tf1-to1)/2*(dyf1-dyo1)+yo1;
        
        // const double to2 = (FLAGS_belt_radius*2)/FLAGS_desired_belt_velocity+FLAGS_acc_y_time+init_time;
        // const double tf2 = (FLAGS_belt_radius*2)/FLAGS_desired_belt_velocity+init_time+FLAGS_acc_y_time*2;
        // const double dyo2 = -FLAGS_desired_belt_velocity;
        // const double dyf2 = 0;
        // const double yf2 = -FLAGS_belt_radius/2-FLAGS_pusher_height;
        // const double yo2 = (to2-tf2)/2*(dyo2-dyf2)+yf2;

        // drake::log()->info("to1: {}\ntf1: {}\nto2: {}\ntf2: {}\nt: {}",to1,tf1,to2,tf2,t);
        // DRAKE_DEMAND(to1 >= 0 && tf1 >= 0 && to2 >= 0 && tf2 >= 0);
        // drake::log()->info("\nyf1: {}\nyf2: {}",yf1,yf2);
        // DRAKE_DEMAND(yo1 > yf1);
        // DRAKE_DEMAND(yo2 > yf2);

        // //Creating the polynomial curve for y_pos
        // double y_pos_des = 0;
        // if(t < tf1){
        //     y_pos_des=FourPolyTraj(t,yo1,yf1,to1,tf1,dyo1,dyf1);
        //     drake::log()->info("accelerating");
        // }
        // else if(t >= tf1 && t < to2){
        //     y_pos_des=-FLAGS_desired_belt_velocity*(t-tf1)+yf1;
        //     drake::log()->info("const velocity");
        // }
        // else if(t >= to2){
        //     y_pos_des=FourPolyTraj(t,yo2,yf2,to2,tf2,dyo2,dyf2);
        //     drake::log()->info("decelerating");
        // }        
        // else{ DRAKE_DEMAND(false); }


        // drake::log()->info("y_pos: {}\ny_vel: {}\nPD: {}",y_pos, y_vel,FLAGS_P*(y_pos_des-y_pos)+FLAGS_D*(-y_vel));
        output->SetAtIndex(x_idx, FLAGS_P*(-FLAGS_belt_length/2-x_pos)+FLAGS_D*(-x_vel));
        // output->SetAtIndex(y_idx, FLAGS_P*(y_pos_des-y_pos)+FLAGS_D*(-y_vel)); //PD controller

        output->SetAtIndex(y_idx, FLAGS_P*(FLAGS_belt_radius/2-y_pos)+FLAGS_D*(-y_vel));
    }
    //Location 2
    else if(location == 2){
        //drake::log()->info("location 2");

        const double to1 = init_time;
        const double tf1 = FLAGS_acc_x_time+init_time;    
        const double dxo1 = 0;
        const double dxf1 = FLAGS_desired_belt_velocity;
        const double xo1 = -FLAGS_belt_length/2;
        const double xf1 =(tf1-to1)/2*(dxf1-dxo1)+xo1;
        
        const double to2 = FLAGS_belt_length/FLAGS_desired_belt_velocity+FLAGS_acc_x_time+init_time;
        const double tf2 = FLAGS_belt_length/FLAGS_desired_belt_velocity+init_time+FLAGS_acc_x_time*2;
        const double dxo2 = FLAGS_desired_belt_velocity;
        const double dxf2 = 0;
        const double xf2 = FLAGS_belt_length/2;
        const double xo2 = (to2-tf2)/2*(dxo2-dxf2)+xf2;

        DRAKE_DEMAND(to1 >= 0 && tf1 >= 0 && to2 >= 0 && tf2 >= 0);

        //Creating the polynomial curve for x_pos
        double x_pos_des = 0;
        if(t < tf1){
            x_pos_des=FourPolyTraj(t,xo1,xf1,to1,tf1,dxo1,dxf1);
        }
        else if(t >= tf1 && t < to2){
            x_pos_des=FLAGS_desired_belt_velocity*(t-tf1)+xf1;
        }
        else if(t >= to2){
            x_pos_des=FourPolyTraj(t,xo2,xf2,to2,tf2,dxo2,dxf2);
        }        
        else{ DRAKE_DEMAND(false); }

        output->SetAtIndex(x_idx, FLAGS_P*(x_pos_des-x_pos)+FLAGS_D*(-x_vel)); //PD controller
        output->SetAtIndex(y_idx, 0);

    }
    //Location 3
    else if(location == 3){
        //drake::log()->info("location 3");

        const double to1 = init_time;
        const double tf1 = FLAGS_acc_y_time+init_time;    
        const double dyo1 = 0;
        const double dyf1 = FLAGS_desired_belt_velocity;
        const double yo1 = -FLAGS_belt_radius;
        const double yf1 =(tf1-to1)/2*(dyf1-dyo1)+yo1;
        
        const double to2 = (FLAGS_belt_radius*2)/FLAGS_desired_belt_velocity+FLAGS_acc_y_time+init_time;
        const double tf2 = (FLAGS_belt_radius*2)/FLAGS_desired_belt_velocity+init_time+FLAGS_acc_y_time*2;
        const double dyo2 = FLAGS_desired_belt_velocity;
        const double dyf2 = 0;
        const double yf2 = FLAGS_belt_radius;
        const double yo2 = (to2-tf2)/2*(dyo2-dyf2)+yf2;

        DRAKE_DEMAND(to1 >= 0 && tf1 >= 0 && to2 >= 0 && tf2 >= 0);

        //Creating the polynomial curve for y_pos
        double y_pos_des = 0;
        if(t < tf1){
            y_pos_des=FourPolyTraj(t,yo1,yf1,to1,tf1,dyo1,dyf1);
        }
        else if(t >= tf1 && t < to2){
            y_pos_des=FLAGS_desired_belt_velocity*(t-tf1)+yf1;
        }
        else if(t >= to2){
            y_pos_des=FourPolyTraj(t,yo2,yf2,to2,tf2,dyo2,dyf2);
        }
        else{ DRAKE_DEMAND(false); }

        output->SetAtIndex(x_idx, 0);
        output->SetAtIndex(y_idx, FLAGS_P*(y_pos_des-y_pos)+FLAGS_D*(-y_vel)); //PD controller

    }
    else{   
        DRAKE_DEMAND(false);
    }
}

template<typename T>
double ConveyorController2<T>::FourPolyTraj(const double time, const double x0, const double xf, const double t0,
                                            const double tfinal, const double dx0, const double dxf) const {
    if(time > tfinal || time < t0){
        //drake::log()->info("time: {}\ntfinal: {}\nt0: {}",time,tfinal,t0);
        //drake::log()->info("time > tfinal: {}\ntime < t0: {}",time > tfinal, time < t0);
        // DRAKE_DEMAND(false);
    }
    const double t = time-t0;
    const double tf = tfinal-t0;
    // const double epsilon = 0.001;
    // if(xf-x0 > tf/2*(dxf-dx0)+epsilon || xf-x0 < tf/2*(dxf-dx0)-epsilon ){
    //     drake::log()->info("xf-x0 != tf/2*(dxf-dx0)\n xf-x0: {}\ntf/2*(dxf-dx0): {}\nxf: {}\nx0: {}\ntf: {}\ndxf: {}\ndx0: {}",xf-x0,tf/2*(dxf-dx0),xf,x0,tf,dxf,dx0);
    //     DRAKE_DEMAND(false);
    // }
    const double a0 = x0;
    const double a1 = dx0;
    const double a2 = 0;
    const double a3=(1/(2*pow(tf,3)))*(20*xf-20*x0-(8*dxf+12*dx0)*tf);
    const double a4=(1/(2*pow(tf,4)))*(30*x0-30*xf+(14*dxf+16*dx0)*tf);
    
    return a0+a1*t+a2*pow(t,2)+a3*pow(t,3)+a4*pow(t,4);
}


//Updates States - this is taken care in multibodyplant so there's no need to do anything here
template<typename T>
void ConveyorController2<T>::DoCalcTimeDerivatives(const systems::Context<T>& ,
    systems::ContinuousState<T>* ) const {}

//Initialize states
template<typename T>
void ConveyorController2<T>::SetDefaultState(const systems::Context<T>&,
                    systems::State<T>* state) const {
    DRAKE_DEMAND(state != nullptr);
    
    Vector4<T> xc0; 
    xc0 << 0,0,0,0;   
    state->get_mutable_continuous_state().SetFromVector(xc0);

    systems::DiscreteValues<double>& discSt =  state->get_mutable_discrete_state();
    discSt[0] = 0;  // start conveyor at location 0
    discSt[1] = 0;  // start conveyor at location 0

    drake::log()->info("set default state2");
}



template class ConveyorController2<double>;

}
}
}
}