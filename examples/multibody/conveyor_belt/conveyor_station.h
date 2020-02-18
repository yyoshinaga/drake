#pragma once

#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace multibody {
namespace conveyor_belt {


template <typename T>
class ConveyorStation : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConveyorStation)
  
    explicit ConveyorStation(double time_step = 0.002);

    void BuildEverything();
  
    const geometry::SceneGraph<T>& get_scene_graph() const {
      return *scene_graph_;
    }

    // These are only valid until Finalize() is called.
    std::unique_ptr<drake::multibody::MultibodyPlant<T>> owned_plant_;
    std::unique_ptr<geometry::SceneGraph<T>> owned_scene_graph_;

    // These are valid for the lifetime of this system.
    std::unique_ptr<drake::multibody::MultibodyPlant<T>> owned_controller_plant_;
    drake::multibody::MultibodyPlant<T>* plant_;
    geometry::SceneGraph<T>* scene_graph_;
    geometry::SourceId source_id_;


  
};

}   // namespace conveyor_belt
}  // namespace multibody
}  // namespace examples
}  // namespace drake
