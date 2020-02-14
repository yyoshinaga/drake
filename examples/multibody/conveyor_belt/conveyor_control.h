#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

template<typename T>
class ConveyorControl : public systems::LeafSystem<T>
{
private:
    /* data */
public:
    ConveyorControl(/* args */);
    ~ConveyorControl();
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConveyorControl); 
    void CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const;
};

}
}
}
}