#include <memory>
#include <gflags/gflags.h>
#include "drake/examples/multibody/conveyor_belt/conveyor_plant.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {


DEFINE_double(belt_length, 1.0, "The length of the belt in meters");
DEFINE_double(belt_radius, 1.0, "The radius of the belt in meters");

DEFINE_double(x_pos_start, FLAGS_belt_length/2, "The x position of the platform in meters");
DEFINE_double(theta_start, 0, "The angle of the platform in rad");
DEFINE_double(belt_velocity, 1.0, "Velocity of belt in meters/seconds");

//Constructor
template<typename T>
ConveyorPlant<T>::ConveyorPlant(std::vector<geometry::FrameId> _frame_ids)
:frame_ids(_frame_ids){

    // A 1D input vector for velocity.
    input_port_index = this->DeclareInputPort(systems::kVectorValued, 1).get_index();

    // Declares the states
    this->DeclareContinuousState(2, 2, 0); //2 position vectors, 0 velocity vectors, 0 miscellaneous vectors

    // A 2D output vector for position: x_pos, V_x, theta, omega
    output_port_index = this->DeclareVectorOutputPort("conveyor_state", systems::BasicVector<T>(4),
                                    &ConveyorPlant::CopyStateOut).get_index();

    // x_pos and theta is sent to multibody plants. Reason is because mbp doesn't want velocities
    pose_output_port = this->DeclareVectorOutputPort("pose_output", systems::BasicVector<T>(2), 
                                    &ConveyorPlant::CalcPoseOutput).get_index();

    // Used for visualization purposes
    this->DeclareAbstractOutputPort("scene_graph_output",geometry::FramePoseVector<T>(),
                                    &ConveyorPlant::CalcFramePoseOutput);                    

    drake::log()->info("conveyor_plant: output_port_index: {}\npose_output_port: {}", output_port_index,pose_output_port);
}

//Destructor
template<typename T>
ConveyorPlant<T>::~ConveyorPlant() {}

//Initialize states
template<typename T>
void ConveyorPlant<T>::SetDefaultState(const systems::Context<T>&,
                    systems::State<T>* state) const {
    DRAKE_DEMAND(state != nullptr);

    Vector4<T> xc0;    
    xc0 << FLAGS_x_pos_start, FLAGS_theta_start, 0 , 0;  // initial state values.
    
    state->get_mutable_continuous_state().SetFromVector(xc0);

    drake::log()->info("set default state");
}

//Output Function
template<typename T>
void ConveyorPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                    systems::BasicVector<T>* output) const {

    drake::log()->info("copy state out beginnning ");

    // Get current state from context.
    const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector(); 

    // Write system output.
    output->set_value(continuous_state_vector.CopyToVector());
    drake::log()->info("copy state out: continuous_state_vector size {}", continuous_state_vector.size());
}

template<typename T>
void ConveyorPlant<T>::CalcPoseOutput(const systems::Context<T>& context,
                                         systems::BasicVector<T>* output) const {
    const auto state_vector = context.get_continuous_state_vector().CopyToVector();
    output->SetAtIndex(0, state_vector[0]);
    output->SetAtIndex(1, state_vector[1]);
}

template<typename T>
void ConveyorPlant<T>::CalcFramePoseOutput(const systems::Context<T>& context, geometry::FramePoseVector<T>* poses) const {
    
    Eigen::Vector3d position_vector;
    Eigen::Vector3d rotation_vector;
    position_vector.setZero();
    rotation_vector.setZero();

    Eigen::VectorXd state_vector = context.get_continuous_state_vector().CopyToVector();

    //This is what affects the model's position and rotation
    position_vector.head<1>() = state_vector.head<1>();     //position about x axis so it grabs the 1st element: X y z alpha beta gamma
    rotation_vector.segment<1>(1) = state_vector.segment<1>(1); //rotation about z axis so it grabs the 6th element: x y z alpha beta GAMMA

    math::RollPitchYawd rotation(rotation_vector);
    math::RigidTransformd pose(rotation, position_vector);

    *poses = {{frame_ids.at(0), pose}, {frame_ids.at(1), pose}, {frame_ids.at(2), pose}};
    drake::log()->info("Calc frame pose output");
    drake::log()->info(context.num_continuous_states());

}

//Updates States 
template<typename T>
void ConveyorPlant<T>::DoCalcTimeDerivatives(const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
    drake::log()->info("do calc time derivative ");

    // Get current state from context.
    const systems::VectorBase<T>& continuous_state_vector =
        context.get_continuous_state_vector();

    // Obtain the structure we need to write into.
    systems::VectorBase<T>& derivatives_vector =
        derivatives->get_mutable_generalized_position();

    // Find out if input velocity should be on or off
    const systems::BasicVector<T>* input_vector =
        this->EvalVectorInput(context, 0);

    // This is the input to the system: if 1, turn on and if 0, turn off
    auto inputVector = input_vector->CopyToVector();

    if(inputVector[0] == 1){
        // drake::log()->info("Conveyor Belt is moving");
   
        // Set positions to variables
        double x_pos = continuous_state_vector.GetAtIndex(0);
        double theta = continuous_state_vector.GetAtIndex(1);
        // drake::log()->info("x_pos, theta: {}, {}",x_pos,theta);

        // Conveyor belt diagram:
        //              0    
        //        _____________
        //   1   (_____________)    3
        //
        //              2

        // bool fullCircle = false;
        // if(theta > M_PI*2){
        //     theta -= M_PI*2;
        //     fullCircle = true;
        // }

        // Approximate the next position and see if it is still within the location bounds
        //Rolling left
        if(x_pos > FLAGS_belt_length/2 && theta < M_PI){
            // drake::log()->info("location 1");
            derivatives_vector.SetAtIndex(0, 0); //V_x
            derivatives_vector.SetAtIndex(1, FLAGS_belt_velocity/FLAGS_belt_radius); //W_z    
        }            
        //Rolling right
        else if(x_pos < -FLAGS_belt_length/2 && theta > 0){
            // drake::log()->info("location 3");
            derivatives_vector.SetAtIndex(0, 0); //V_x
            derivatives_vector.SetAtIndex(1, -FLAGS_belt_velocity/FLAGS_belt_radius); //W_z
        }
        else{
            //Moving rightwards
            if(theta >= M_PI){
                // drake::log()->info("location 2");
                derivatives_vector.SetAtIndex(0, -FLAGS_belt_velocity); //V_x
                derivatives_vector.SetAtIndex(1, 0); //W_z
            }
            //Moving leftwards
            else if(theta <= 0){
                // drake::log()->info("location 0");
                derivatives_vector.SetAtIndex(0, FLAGS_belt_velocity); //V_x
                derivatives_vector.SetAtIndex(1, 0); //W_z
            }

            else{
                drake::log()->info("location ERROR");
                
            }
        }
    }    
    else{
        drake::log()->info("Conveyor Belt has stopped {}", inputVector[0]);
        derivatives_vector.SetAtIndex(0, 0);    //V_x = 0
        derivatives_vector.SetAtIndex(1, 0);    //W = 0
    }
    drake::log()->info("finished do calc derivative");
}


template class ConveyorPlant<double>; //Forces the instantiation of the template with a double

}  // namespace conveyor
}  // namespace multibody
}  // namespace examples
}  // namespace drake
