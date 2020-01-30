#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace yaskawa {

constexpr int kYaskawaArmNumJoints = 6;

/// Returns the maximum joint velocities provided by Yaskawa.
/// @return Maximum joint velocities (rad/s).
VectorX<double> get_yaskawa_max_joint_velocities();

extern const double kYaskawaLcmStatusPeriod;

}  // namespace yaskawa
}  // namespace manipulation
}  // namespace drake
