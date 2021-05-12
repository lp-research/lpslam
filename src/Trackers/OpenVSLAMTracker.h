#pragma once

#include "DataTypes/Space.h"
#include "Trackers/OpenVSLAMTrackerBase.h"
#include "DataTypes/CameraQueue.h"
#include "Utils/ManagedThread.h"

#include <openvslam/system.h>
#include <openvslam/config.h>

#ifdef LPSLAM_BUILD_OPENVSLAM_PANGOLIN
#include <pangolin_viewer/viewer.h>
#endif

#include <string>
#include <memory>
#include <optional>

namespace LpSlam {

class OpenVSLAMTracker : public OpenVSLAMTrackerBase {
public:
    OpenVSLAMTracker();

    TrackerBase::ProcessImageResult processImage(CameraQueueEntry & cam,
        std::optional<GlobalStateInTime> navResultOdom = std::nullopt,
        std::optional<GlobalStateInTime> navResultMap = std::nullopt,
        std::vector<SensorQueueEntry> const& sensorValues = {}) override;

    std::string type() override {
        // using not the name OpenVSLAM here to not give away
        // which library we are using in the configuration
        return "VSLAMMono";
    }

    bool start(SensorQueue & sensorQ) override;

    bool stop() override;

public:

#ifdef LPSLAM_BUILD_OPENVSLAM_PANGOLIN
    //std::unique_ptr<pangolin_viewer::viewer> m_viewer;
    pangolin_viewer::viewer * m_viewer;

    struct PangolinThreadParams
    {
        pangolin_viewer::viewer * viewer;
    };

    ManagedThread<PangolinThreadParams> m_pangolinWorker;
#endif
    int m_imageTracked = 0;
    std::optional<TimeStamp> m_firstImageTimestamp;
    TimeStamp m_lastEmit;
    SensorQueue * m_sensorQ = nullptr;
};

}