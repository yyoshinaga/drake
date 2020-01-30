#include "drake/manipulation/yaskawa/yaskawa_constants.h"

namespace drake {
namespace manipulation {
namespace yaskawa {

VectorX<double> get_iiwa_max_joint_velocities() {
  // These are the maximum joint velocities given on the website for yaskawa sgp25 series
  // That document is available here:
  // https://www.motoman.com/en-us/products/robots/industrial/assembly-handling/gp-series/gp25
  return (VectorX<double>(7) << 
          3.14159,                         // 180°/s in rad/s
          2.70526,                         // 155°/s in rad/s ~ TODO: Fix this number to +155/-105
          2.79253,                         // 160°/s in rad/s ~ TODO: Fix this number to +160/-86
          3.49066,                         // 200°/s in rad/s
          2.61799,                         // 150°/s in rad/s
          7.94125)                         // 455°/s in rad/s
      .finished();
}

// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the yaskawa's control
// cabinet.
const double kYaskawaLcmStatusPeriod = 0.005;  //TODO: find out the correct number

}  // namespace yaskawa
}  // namespace manipulation
}  // namespace drake
