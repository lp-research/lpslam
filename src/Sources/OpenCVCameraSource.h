#pragma once

#include "DataTypes/Space.h"
#include "Trackers/TrackerBase.h"
#include "Utils/ManagedThread.h"
#include "DataTypes/CameraQueue.h"
#include "Sources/ImageSourceBase.h"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <string>

namespace LpSlam {

/**
 * If you run into trouble because another camera on the system intereferes, you
 * can use these instructions to disable the other camera:
 * 
 * https://askubuntu.com/questions/747212/how-to-disable-integrated-webcam-on-ubuntu
 * 
 * To list the available resolutions and frame rates for a camera run:
 * v4l2-ctl --list-formats-ext
 */

class OpenCVCameraSource : public ImageSourceBase{
public:

    OpenCVCameraSource();

    bool start(CameraQueue &) override;

    void stop() override;

private:
    struct WorkerThreadParams {
        CameraQueue & camQ;
        cv::VideoCapture * videoCapture;
        LpSlamImageStructure image_structure;
        uint32_t & frameNumber;
        bool convertToGrayscale;
        int targetFps;
        uint64_t & lastTimestamp;
    };

    ManagedThread<WorkerThreadParams> m_worker;
    std::unique_ptr<cv::VideoCapture> m_videoCapture;
    uint32_t m_frameNumber = 0;
    uint64_t m_lastTimestamp = 0;

    const std::string m_configGrayscale = "grayscale";
    const std::string m_configCameraNumber = "camera_number";
    const std::string m_configWidth = "width";
    const std::string m_configHeight = "height";
    const std::string m_configStereoMode = "stereo_mode";
    const std::string m_configFps = "fps";
    const std::string m_configExposure = "exposure";
    const std::string m_configGain = "gain";
    const std::string m_configOpenCVLoggingVerbose = "opencv_logging_verbose";
    const std::string m_configTargetFps = "target_fps";
};

}