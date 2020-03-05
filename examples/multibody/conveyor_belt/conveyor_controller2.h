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
class ConveyorController2 : public systems::LeafSystem<T>
{
  private:
    std::vector<geometry::FrameId> frame_ids;

    int state_input_port_index = -1;
    int acc_output_port_index = -1;
    int position_output_port_index = -1;

  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConveyorController2);
    ConveyorController2(std::vector<geometry::FrameId> _frame_ids);
    ~ConveyorController2();

    void CopyPositionOut(const systems::Context<T>& context,
                        systems::BasicVector<T>* output) const;

    void CalcAccOutput(const systems::Context<T>& context,
                        systems::BasicVector<T>* output) const;

    void SetDefaultState(const systems::Context<T>&,
                          systems::State<T>* state) const;

    void DoCalcTimeDerivatives(const systems::Context<T>& context,
                                systems::ContinuousState<T>* derivatives) const;

    void Update(const systems::Context<T>& context,
                systems::DiscreteValues<T>* updates) const;

    double FourPolyTraj(const double time, 
                      const double x0, 
                      const double xf, 
                      const double t0, 
                      const double tfinal, 
                      const double dx0, 
                      const double dxf) const;


};

}
}
}
}