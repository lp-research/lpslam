#pragma once

#include "Interface/LpSlamTypes.h"
#include "Utils/LockedAccess.h"

#include <optional>
#include <map>

namespace LpSlam {

    class CameraRegistry {
    public:
        void setConfiguration(LpSlamCameraConfiguration const& config);
        std::optional<LpSlamCameraConfiguration> getConfiguration(LpSlamCameraNumber num);
    
    private:
        std::map<LpSlamCameraNumber, LpSlamCameraConfiguration> m_cams;
    };

    typedef LockedAccess<CameraRegistry> CameraRegistryAccess;
}