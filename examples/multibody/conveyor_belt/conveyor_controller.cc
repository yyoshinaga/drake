#include <memory>
#include <gflags/gflags.h>
#include "drake/examples/multibody/conveyor_belt/conveyor_controller.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

DEFINE_double(belt_length, 1.0, "The length of the belt in meters");
DEFINE_double(belt_radius, 1.0, "The radius of the belt in meters");

DEFINE_double(x_pos_start, FLAGS_belt_length/2, "The x position of the platform in meters");
DEFINE_double(theta_start, 0, "The angle of the platform in rad");
DEFINE_double(belt_velocity, 1.0, "Velocity of belt in meters/seconds");

template<typename T>
ConveyorController<T>::ConveyorController(std::vector<geometry::FrameId> _frame_ids)
:frame_ids(_frame_ids){
    
    // A 1D input vector for velocity.
    input_port_index = this->DeclareInputPort(systems::kVectorValued, 4).get_index();

    // //Control output includes one variable, which is a trigger for the conveyor to move.
    // this->DeclareVectorOutputPort(systems::BasicVector<T>(2), &ConveyorController::CopyStateOut);

    // Declares the states
    this->DeclareContinuousState(2, 2, 0); //2 position vectors, 0 velocity vectors, 0 miscellaneous vectors

    // A 2D output vector for position: x_pos, V_x, theta, omega
    output_port_index = this->DeclareVectorOutputPort("conveyor_state", systems::BasicVector<T>(2),
                                    &ConveyorController::CopyStateOut).get_index();  //0

    // x_pos and theta is sent to multibody plants. Reason is because mbp doesn't want velocities
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

    // Write system output.
    // output->set_value(continuous_state_vector.CopyToVector());
    output->SetAtIndex(0, continuous_state_vector.CopyToVector()[0]);//state_vector[3]);
    output->SetAtIndex(1, continuous_state_vector.CopyToVector()[1]); //state_vector[2]);
    // //Controller can be set 0 or 1. Off or On.
    // Vector1<double> vec(1);
    // vec << 1;
    // output->set_value(vec);
}

template<typename T>
void ConveyorController<T>::CalcPoseOutput(const systems::Context<T>& context ,
                                         systems::BasicVector<T>* output) const {
    //  const systems::VectorBase<T>& continuous_state_vector = context.get_continuous_state_vector();

    // Get the states from the plant
    const systems::BasicVector<T>* input_vector =
        this->EvalVectorInput(context, 0);
    auto inputVector = input_vector->CopyToVector();






        // Set positions to variables
        double x_pos = inputVector[0];
        double theta = inputVector[1];

        // Conveyor belt diagram:
        //              0    
        //        _____________
        //   1   (_____________)    3
        //
        //              2

        
        theta = fmod(theta,M_PI*2);
        if(theta < 0){
            theta += M_PI*2;
        }
        // if(theta < 0)
        drake::log()->info("start theta: deg {}  x_pos: {}", theta*180/M_PI, x_pos);
        // DRAKE_DEMAND(theta >= 0);

        // Approximate the next position and see if it is still within the location bounds
        //Rolling left
        if(x_pos > FLAGS_belt_length/2 && theta >= 0 && theta < M_PI){
            if(theta <= M_PI/2){
                output->SetAtIndex(1, FLAGS_belt_velocity/FLAGS_belt_radius); //W_z
            }
            else{
                output->SetAtIndex(1, -FLAGS_belt_velocity/FLAGS_belt_radius); //W_z
            }
            drake::log()->info("location 1");
            output->SetAtIndex(0, 0); //V_x
        }            
        //Rolling right
        else if(x_pos < -FLAGS_belt_length/2 && theta >= M_PI){
            if(theta <= 3*M_PI/2){
                output->SetAtIndex(1, FLAGS_belt_velocity/FLAGS_belt_radius); //W_z
            }
            else{
                output->SetAtIndex(1, -FLAGS_belt_velocity/FLAGS_belt_radius); //W_z
            }
            drake::log()->info("location 3");
            output->SetAtIndex(0, 0); //V_x
        }
        else{
            //Moving rightwards
            if(x_pos > -FLAGS_belt_length/2 && x_pos < FLAGS_belt_length/2){
                if(theta >= M_PI){
                    if(x_pos <= 0){
                        output->SetAtIndex(0, -FLAGS_belt_velocity); //V_x
                    }
                    else{
                        output->SetAtIndex(0, FLAGS_belt_velocity); //V_x
                    }
                    drake::log()->info("location 2");
                    output->SetAtIndex(1, 0); //W_z
                }
                //Moving leftwards
                else if(theta >= 0){
                    if(x_pos >= 0){
                        output->SetAtIndex(0, FLAGS_belt_velocity); //V_x
                    }
                    else{
                        output->SetAtIndex(0, -FLAGS_belt_velocity); //V_x
                    }
                    drake::log()->info("location 0");
                    output->SetAtIndex(1, 0); //W_z
                }
                else{
                    drake::log()->info("location 1 ERROR - theta: {}  x_pos: {}",theta,x_pos);
                }
            }
            else{
                drake::log()->info("location 2 ERROR - theta: {}  x_pos: {}",theta,x_pos);
            }
        }
    // }    
    // else{
    //     drake::log()->info("Conveyor Belt has stopped {}", inputVector[0]);
    //     output->.SetAtIndex(1, 0);    //V_x = 0
    //     output->.SetAtIndex(0, 0);    //W = 0
    // }











    drake::log()->info("input1: {}\n intput2: {}\n intput3: {}\n intput4: {}", inputVector[0], inputVector[1], inputVector[2], inputVector[3]);
}

//Updates States 
template<typename T>
void ConveyorController<T>::DoCalcTimeDerivatives(const systems::Context<T>& ,
    systems::ContinuousState<T>* derivatives) const {}

//Initialize states
template<typename T>
void ConveyorController<T>::SetDefaultState(const systems::Context<T>&,
                    systems::State<T>* state) const {
    DRAKE_DEMAND(state != nullptr);

    Vector4<T> xc0;    
    xc0 << FLAGS_theta_start, FLAGS_x_pos_start, 0 , 0;  // initial state values.
    
    state->get_mutable_continuous_state().SetFromVector(xc0);

    drake::log()->info("set default state2");
}

template class ConveyorController<double>;

}
}
}
}