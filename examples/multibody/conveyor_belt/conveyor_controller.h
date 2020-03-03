#pragma once
#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/input_port.h"

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/scene_graph.h"
#include <cmath>        // std::atan2
#include <math.h>

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {

template<typename T>
class ConveyorController : public systems::LeafSystem<T>
{
  private:
    std::vector<geometry::FrameId> frame_ids;

    int input_port_index = -1;
    int output_port_index = -1;
    int pose_output_port = -1;

  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConveyorController);
    ConveyorController(std::vector<geometry::FrameId> _frame_ids);
    ~ConveyorController();


    void CopyStateOut(const systems::Context<T>& context,
                        systems::BasicVector<T>* output) const;

    void CalcPoseOutput(const systems::Context<T>& context,
                        systems::BasicVector<T>* output) const;

    void SetDefaultState(const systems::Context<T>&,
                          systems::State<T>* state) const;

    void DoCalcTimeDerivatives(const systems::Context<T>& context,
                                systems::ContinuousState<T>* derivatives) const;

    void Update(const systems::Context<T>& context,
                systems::DiscreteValues<T>* updates) const;


};

}
}
}
}