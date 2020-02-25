/**
 * @file
 *
 * Constants defined in this file are for the Schunk WSG gripper modeled in
 * models/schunk_wsg_50.sdf.
 */

#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace manipulation {
namespace yaskawa_conveyor_belt_dof1 {

constexpr int kEndEffectorNumActuators = 4;
constexpr int kEndEffectorNumPositions = 4;
constexpr int kEndEffectorNumVelocities = 4;

// TODO(russt): These constants are predicated on the idea that we can map
// one finger's state => gripper state.  We should prefer using
// MakeMultibodyStateToBeltState and MakeMultibodyForceToBeltForce instead.
// But I cannot deprecate them yet without performing surgery on the old
// controller.
constexpr int kEndEffectorPositionIndex = 0;
constexpr int kEndEffectorVelocityIndex =
    kEndEffectorNumPositions + kEndEffectorPositionIndex;

// TODO(sammy): need to double check this.
constexpr double kEndEffectorLcmStatusPeriod = 0.05;

/**
 * Returns the position vector corresponding to the open position of the
 * gripper.
 */
template <typename T>
VectorX<T> GetEndEffectorOpenPosition() {
  // clang-format off
  return (VectorX<T>(kEndEffectorNumPositions) <<
      -0.0550667,
       0.0550667,0,0).finished();
  // clang-format on
}

/// Extract the distance between the fingers (and its time derivative) out
/// of the plant model which pretends the two fingers are independent.
template <typename T>
std::unique_ptr<systems::MatrixGain<T>>
MakeMultibodyStateToBeltStateSystem() {
  Eigen::Matrix<double, 6, 8> D;  //wsg_state_measured of size 6, then 4*2 states
  // clang-format off
  //THERE MUST BE 6 STATES
  D << -1, 1, 0, 0, 0, 0, 0, 0,
        0, 0, -1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1;
  // clang-format on
  return std::make_unique<systems::MatrixGain<T>>(D);
}

template <typename T>
class MultibodyForceToBeltForceSystem : public systems::VectorSystem<T> {
 public:
  MultibodyForceToBeltForceSystem()
      : systems::VectorSystem<T>(
            systems::SystemTypeTag<MultibodyForceToBeltForceSystem>{}, 4, 2) {}

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit MultibodyForceToBeltForceSystem(
      const MultibodyForceToBeltForceSystem<U>&)
      : MultibodyForceToBeltForceSystem<T>() {}

  void DoCalcVectorOutput(
      const systems::Context<T>&,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
    unused(state);
    // gripper force = abs(-finger0 + finger1).
    using std::abs;
    (*output)(0) = abs(input(0) - input(1));
  }
};

/// Extract the gripper measured force from the generalized forces on the two
/// fingers.
template <typename T>
std::unique_ptr<systems::VectorSystem<T>>
MakeMultibodyForceToBeltForceSystem() {
  return std::make_unique<MultibodyForceToBeltForceSystem<T>>();
}


}  // namespace yaskawa_conveyor_belt_dof1
}  // namespace manipulation
}  // namespace drake
