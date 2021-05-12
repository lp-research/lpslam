#ifndef LPSLAM_SUPPORT
#define LPSLAM_SUPPORT

#include "Interface/LpSlamTypes.h"

namespace LpSlam {
namespace support{

inline LpSlamStatus createLpSlamStatus() {
    LpSlamStatus status;
    status.localization = LpSlamLocalization_Off;
    status.feature_points = 0;
    status.key_frames = 0;
    status.frame_time = 0.0;
    status.fps = 0.0;

    return status;
}

}
}

#endif