#include <memory>
#include <gflags/gflags.h>
#include "drake/examples/multibody/conveyor_belt/conveyor_plant.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

DEFINE_double(belt_length, 10.0, "The length of the belt in meters");
DEFINE_double(belt_radius, 1.0, "The radius of the belt in meters");

DEFINE_double(x_pos_start, FLAGS_belt_length/2, "The x position of the platform in meters");
DEFINE_double(theta_start, 0, "The angle of the platform in rad");
DEFINE_double(belt_velocity, 3.0, "Velocity of belt in meters/seconds");


//Constructor
template<typename T>
ConveyorPlant<T>::ConveyorPlant(std::vector<geometry::FrameId> _frame_ids)
:frame_ids(_frame_ids){
  // A 1D input vector for velocity.
  input_port_index = this->DeclareInputPort(systems::kVectorValued, 1).get_index();

//   this->DeclareVectorInputPort( systems::BasicVector<T>(1));
  this->DeclareContinuousState(2, 0, 0); //2 position vectors, 0 velocity vectors, 0 miscellaneous vectors

  // A 2D output vector for position: x_pos, theta
  output_port_index = this->DeclareVectorOutputPort("conveyor_state", systems::BasicVector<T>(2),
                                &ConveyorPlant::CopyStateOut).get_index();

  this->DeclareAbstractOutputPort("scene_graph_output",geometry::FramePoseVector<T>(),
                                &ConveyorPlant::CalcFramePoseOutput);
}

//Destructor
template<typename T>
ConveyorPlant<T>::~ConveyorPlant() {}

//Initialize states
template<typename T>
void ConveyorPlant<T>::SetDefaultState(const systems::Context<T>&,
                    systems::State<T>* state) const {
    DRAKE_DEMAND(state != nullptr);
    Vector2<T> xc0;    drake::log()->info("set default state 1");

    xc0 << FLAGS_x_pos_start, FLAGS_theta_start;  // initial state values.
    state->get_mutable_continuous_state().SetFromVector(xc0);
}

//Output Function
template<typename T>
void ConveyorPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                    systems::BasicVector<T>* output) const {
    // Get current state from context.
    const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector(); 
    drake::log()->info("copy state out 2 ");

    // Write system output.
    output->set_value(continuous_state_vector.CopyToVector());
}

template<typename T>
void ConveyorPlant<T>::CalcFramePoseOutput(const systems::Context<T>& context, geometry::FramePoseVector<T>* poses) const {
    Eigen::Vector3d position_vector;
    Eigen::Vector3d rotation_vector;
    position_vector.setZero();
    rotation_vector.setZero();

    Eigen::VectorXd state_vector = context.get_continuous_state_vector().CopyToVector();

    position_vector.head<2>() = state_vector.head<2>();
    rotation_vector.tail<1>() = state_vector.tail<1>();

    math::RollPitchYawd rotation(rotation_vector);
    math::RigidTransformd pose(rotation, position_vector);
    drake::log()->info("Calc frame pose output 3");

    *poses = {{frame_ids.at(0), pose}, {frame_ids.at(1), pose}, {frame_ids.at(2), pose}, {frame_ids.at(3), pose}};
}

//Updates States 
template<typename T>
void ConveyorPlant<T>::DoCalcTimeDerivatives(const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
    drake::log()->info("do calc time derivative 4 ");

    // Get current state from context.
    const systems::VectorBase<T>& continuous_state_vector =
        context.get_continuous_state_vector();

    // Obtain the structure we need to write into.
    systems::VectorBase<T>& derivatives_vector =
        derivatives->get_mutable_vector();

    // Find out if input velocity should be on or off
    const systems::BasicVector<T>* input_vector =
        this->EvalVectorInput(context, 0);

    auto inputVector = input_vector->CopyToVector();
    if(inputVector[0] == 1){
        drake::log()->info("Conveyor Belt is moving");
    }    
    else{
        drake::log()->info("Conveyor Belt has stopped {}", inputVector[0]);
    }

    // Set positions to variables
    double x_pos = continuous_state_vector.GetAtIndex(0);
    double theta = continuous_state_vector.GetAtIndex(1);

    int location = 0; 

    // Approximate the next position and see if it is still within the location bounds

    if(location == 0){
        if(x_pos <= -FLAGS_belt_length/2){
            location = 1;
        }
    }
    else if(location == 1){
        if(theta >= 3*M_PI/4){
            location = 2;
        }
    }
    else if(location == 2){
        if(x_pos >= FLAGS_belt_length/2){
            location = 3;
        }
    }
    else{
        if(theta >= M_PI/2){
            location = 0;
        }
    }

    drake::log()->info("middle of do calc derivative");

    switch(location){
        case 0: 
            derivatives_vector.SetAtIndex(0, -FLAGS_belt_velocity); //V_x
            derivatives_vector.SetAtIndex(0, 0); //W
            break;
        case 1:
            derivatives_vector.SetAtIndex(0, 0); //V_x = radius*(-sin(theta))*(vel/radius) = (-sin(theta))*vel
            derivatives_vector.SetAtIndex(0, FLAGS_belt_velocity/FLAGS_belt_radius); //W
            break;
        case 2:
            derivatives_vector.SetAtIndex(0, FLAGS_belt_velocity); //V_x
            derivatives_vector.SetAtIndex(0,  0); //W
            break;
        case 3:
            derivatives_vector.SetAtIndex(0, 0); //V_x
            derivatives_vector.SetAtIndex(0, FLAGS_belt_velocity/FLAGS_belt_radius); //W
            break;
    }
    
}


template class ConveyorPlant<double>; //Forces the instantiation of the template with a double

}  // namespace conveyor
}  // namespace multibody
}  // namespace examples
}  // namespace drake
