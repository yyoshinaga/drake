#include "drake/examples/multibody/conveyor_belt/conveyor_control.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

template<typename T>
ConveyorControl<T>::ConveyorControl(/* args */)
{
    this->DeclareVectorOutputPort(systems::BasicVector<T>(1), &ConveyorControl::CopyStateOut);
}

template<typename T>
ConveyorControl<T>::~ConveyorControl() {}

template<typename T>
void ConveyorControl<T>::CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
    // Get current state from context.
    const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector(); 

    drake::log()->info(continuous_state_vector.size());

    VectorX<double> vec(1);
    vec << 1;
    output->set_value(vec);
}

template class ConveyorControl<double>;

}
}
}
}