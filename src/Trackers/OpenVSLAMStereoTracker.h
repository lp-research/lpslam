#pragma once

#include "DataTypes/Space.h"
#include "Trackers/TrackerBase.h"
#include "DataTypes/CameraQueue.h"
#include "Utils/ManagedThread.h"
#include "Utils/ImageProcessing.h"

#include "OpenVSLAMTrackerBase.h"

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/data/laser2d.h>

#ifdef LPSLAM_BUILD_OPENVSLAM_PANGOLIN
#include <pangolin_viewer/viewer.h>
#endif

#include <vector>
#include <string>
#include <memory>
#include <optional>

namespace LpSlam {

class OpenVSLAMStereoTracker : public OpenVSLAMTrackerBase {
public:
    OpenVSLAMStereoTracker();

    TrackerBase::ProcessImageResult processImage(CameraQueueEntry & cam,
        std::optional<GlobalStateInTime> navResultOdom = std::nullopt,
        std::optional<GlobalStateInTime> navResultMap = std::nullopt,
        std::vector<SensorQueueEntry> const& sensorValues = {}) override;

    std::string type() override {
        // using not the name OpenVSLAM here to not give away
        // which library we are using in the configuration
        return "VSLAMStereo";
    }

    bool start(SensorQueue & sensorQ) override;

    bool stop() override;

    void addLaserScan(GlobalStateInTime origin,
        float * ranges, size_t rangeCount,
        float start_range,
        float end_range,
        float start_angle,
        float end_angle,
        float increment,
        float range_threshold) override;


    std::optional<unsigned long> mappingGetMapRawSize() override;

    std::optional<LpMapInfo> mappingGetMapRaw( int8_t * map, std::size_t mapSize) override;

private:

    struct LaserBuffer {
        GlobalStateInTime origin;
        openvslam::data::laser2d laser_data;
    };

    LaserBuffer m_laserBuffer;
    std::mutex m_laserBufferMutex;

    int m_imageTracked = 0;
    std::optional<TimeStamp> m_firstImageTimestamp;
    ImageProcessing::Undistort m_undistortLeft;
    ImageProcessing::Undistort m_undistortRight;
};

}