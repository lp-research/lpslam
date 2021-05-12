#ifndef LPGFSLAM_CONFIGURATION_H
#define LPGFSLAM_CONFIGURATION_H

/**
* Interface for the LpgfSlam Manage
*/

#include "LpSlamTypes.h"
#include "LpSlamExport.h"

class LPSLAM_EXPORT LpSlamConfiguration {
public:
    LpSlamCameraConfiguration createDefaultCameraConfiguration();
};

#endif