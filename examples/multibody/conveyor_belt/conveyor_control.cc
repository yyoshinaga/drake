#include "drake/examples/multibody/conveyor_belt/conveyor_control.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

template<typename T>
ConveyorControl<T>::ConveyorControl()
{
    //Control output includes one variable, which is a trigger for the conveyor to move.
    this->DeclareVectorOutputPort(systems::BasicVector<T>(1), &ConveyorControl::CopyStateOut);
}

template<typename T>
ConveyorControl<T>::~ConveyorControl() {}

template<typename T>
void ConveyorControl<T>::CopyStateOut(const systems::Context<T>&, systems::BasicVector<T>* output) const {

    //Controller can be set 0 or 1. Off or On.
    Vector1<double> vec(1);
    vec << 1;
    output->set_value(vec);
}


template class ConveyorControl<double>;

}
}
}
}