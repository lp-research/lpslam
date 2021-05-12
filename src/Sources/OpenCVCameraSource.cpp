#include "OpenCVCameraSource.h"

#include <spdlog/spdlog.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utils/logger.hpp>


using namespace LpSlam;

OpenCVCameraSource::OpenCVCameraSource() : 
    m_worker( [](OpenCVCameraSource::WorkerThreadParams params) -> bool {

    cv::Mat frame;
    SPDLOG_DEBUG("Getting next frame from OpenCV source ...");
    if (!params.videoCapture->read(frame)) {
        spdlog::error("Cannot capture video frame");
        return false;
    }

    auto cvTimestamp = params.videoCapture->get(cv::CAP_PROP_POS_MSEC);

    // check if we need to scale fps target because ZED camera was not
    // able to set the FPS on driver level ^^
    if (params.targetFps > 0) {
        // convert to ms
        const uint64_t frameTime = 1.0 / double(params.targetFps) * std::pow(10,3);
        if ((cvTimestamp - params.lastTimestamp) < frameTime) {
            // don't continue, no new frame yet
            return true;
        }
        params.lastTimestamp = cvTimestamp;
    }

    params.frameNumber++;

    // convert to grayscale before we do anything else, saves us
    // memory bandwidth later on
    cv::Mat cvFrameColorConverted;
    if (params.convertToGrayscale) {
        cv::cvtColor(frame, cvFrameColorConverted, cv::COLOR_BGR2GRAY);
    } else {
        cvFrameColorConverted = frame;
    }

    // vive pro, images are on top of each other
    cv::Mat croppedFrameLower;
    cv::Mat croppedFrameUpper;

    if (params.image_structure == LpSlamImageStructure_OneImage) {
        croppedFrameUpper = cvFrameColorConverted;
    } else if (params.image_structure == LpSlamImageStructure_Stereo_LeftTop_RightBottom) {
        // this is left cam
        croppedFrameLower = cvFrameColorConverted(cv::Rect(0, cvFrameColorConverted.rows/2, cvFrameColorConverted.cols, cvFrameColorConverted.rows/2));
        // this is right cam
        croppedFrameUpper = cvFrameColorConverted(cv::Rect(0, 0, cvFrameColorConverted.cols, cvFrameColorConverted.rows/2));
    } else if (params.image_structure == LpSlamImageStructure_Stereo_LeftLeft_RightRight) {
        // valve index, images are next to each other
        // this is left cam
        croppedFrameLower = cvFrameColorConverted(cv::Rect(0, 0, cvFrameColorConverted.cols/2, cvFrameColorConverted.rows));
        // this is right cam
        croppedFrameUpper = cvFrameColorConverted(cv::Rect(cvFrameColorConverted.cols/2, 0, cvFrameColorConverted.cols/2, cvFrameColorConverted.rows));
    } else {
        spdlog::error("Image structure {0} not supported", params.image_structure);
        return false;
    }

    // for ZED its 12 cm camera distance
    TrackerCoordinateSystemBase base_left;
    TrackerCoordinateSystemBase base_right;

    // the eye line of the HMD headset is in our y-axis
    // valve index
    base_left.position.value[1] = -0.5 * 0.12;
    base_right.position.value[1] = 0.5 * 0.12;

    if (params.image_structure != LpSlamImageStructure_OneImage) {
        params.camQ.push( CameraQueueEntry( std::chrono::high_resolution_clock::now(),
            0, // left cam
            base_left,
            croppedFrameLower,
            1, // right cam
            base_right,
            croppedFrameUpper
            ));
    } else {
        TrackerCoordinateSystemBase base;
        params.camQ.push( CameraQueueEntry( std::chrono::high_resolution_clock::now(),
            base,
            croppedFrameUpper
            ));
    }

       return true; 
    } ) {

    getConfigOptions().optional(m_configCameraNumber, int(0));
    getConfigOptions().optional(m_configGrayscale, true);
    getConfigOptions().optional(m_configWidth, int(0));
    getConfigOptions().optional(m_configHeight, int(0));
    getConfigOptions().optional(m_configStereoMode, std::string(""));
    getConfigOptions().optional(m_configFps, int(0));
    getConfigOptions().optional(m_configExposure, int(0));
    getConfigOptions().optional(m_configGain, int(0));
    getConfigOptions().optional(m_configOpenCVLoggingVerbose, false);
    getConfigOptions().optional(m_configTargetFps, int(0));
}

bool OpenCVCameraSource::start(CameraQueue & q) {
    if (getConfigOptions().getBool(m_configOpenCVLoggingVerbose)) {
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_VERBOSE);
    } else {
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);
    }

    auto camNum = getConfigOptions().getInteger(m_configCameraNumber);
    spdlog::info("Opening OpenCV video capture interface number {0}", camNum);

    const int maxRetry = 5;

    for (int retryCount = 0; retryCount < maxRetry; retryCount++) {
        m_videoCapture = std::make_unique<cv::VideoCapture>(camNum);
        if (!m_videoCapture->isOpened()) {
            spdlog::debug("Cannot open OpenCV video capture with {0}th try", retryCount);
            m_videoCapture = nullptr;
        } else {
            spdlog::debug("Opened OpenCV video capture with {0}th try", retryCount);
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (m_videoCapture == nullptr) {
        spdlog::error("Cannot open OpenCV video capture");
        return false;
    }

    spdlog::info("OpenCV backend name {0}", m_videoCapture->getBackendName());

    // from ZED example code
    // https://github.com/stereolabs/zed-opencv-native/blob/master/cpp/src/main.cpp
    LpSlamImageStructure imageStruct = LpSlamImageStructure_OneImage;
    auto stereo_mode = getConfigOptions().getString(m_configStereoMode);
    if (stereo_mode == "horizontal") {
        imageStruct = LpSlamImageStructure_Stereo_LeftLeft_RightRight;
    } else if (stereo_mode == "vertical") {
        imageStruct = LpSlamImageStructure_Stereo_LeftTop_RightBottom;
    } else if (stereo_mode == "") {
        // ignore
    } else {
        spdlog::error("stereo camera mode {0} not supported", stereo_mode);
        return false;
    }

    const auto width = getConfigOptions().getInteger(m_configWidth);
    const auto height = getConfigOptions().getInteger(m_configHeight);
    const auto fps = getConfigOptions().getInteger(m_configFps);
    const auto exposure = getConfigOptions().getInteger(m_configExposure);
    const auto gain = getConfigOptions().getInteger(m_configGain);
    const auto targetFps = getConfigOptions().getInteger(m_configTargetFps);

    spdlog::info("Default image resolution {0} x {1}",
        m_videoCapture->get(cv::CAP_PROP_FRAME_WIDTH),
        m_videoCapture->get(cv::CAP_PROP_FRAME_HEIGHT));
    spdlog::info("Default camera fps: {0}",
        m_videoCapture->get(cv::CAP_PROP_FPS));

    // configuring different resolution does not seem to work
    // with ZED camera ?!
    int reqWidth = 0;
    int reqHeight = 0;
    if (width > 0) {
        reqWidth = width * (imageStruct == LpSlamImageStructure_Stereo_LeftLeft_RightRight ? 2 : 1);
        m_videoCapture->set(cv::CAP_PROP_FRAME_WIDTH, reqWidth);
    }
    if (height > 0) {
        reqHeight = height * (imageStruct == LpSlamImageStructure_Stereo_LeftTop_RightBottom ? 2 : 1);
        m_videoCapture->set(cv::CAP_PROP_FRAME_HEIGHT, reqHeight);
    }

    if (fps > 0) {
        m_videoCapture->set(cv::CAP_PROP_FPS, fps);
    }

    if (exposure > 0) {
        m_videoCapture->set(cv::CAP_PROP_EXPOSURE, exposure);
    }

    if (gain > 0) {
        m_videoCapture->set(cv::CAP_PROP_GAIN, gain);
    }

    spdlog::info("Selected image resolution {0} x {1}",
        m_videoCapture->get(cv::CAP_PROP_FRAME_WIDTH),
        m_videoCapture->get(cv::CAP_PROP_FRAME_HEIGHT));

    if ( reqHeight > 0 && reqWidth > 0) {
        if ( (reqHeight != m_videoCapture->get(cv::CAP_PROP_FRAME_HEIGHT)) ||
            (reqWidth != m_videoCapture->get(cv::CAP_PROP_FRAME_WIDTH)) ) {
                spdlog::error("Requested resolution could not be set. Ensure the camera is connected to a fast enough IO port (USB3 for example). Resolution now is {0} x {1}, target was {2} x {3}",
                    m_videoCapture->get(cv::CAP_PROP_FRAME_WIDTH), m_videoCapture->get(cv::CAP_PROP_FRAME_HEIGHT),
                    reqWidth, reqHeight);
                return false;
            }
    }

    spdlog::info("Selected fps {0}",
        m_videoCapture->get(cv::CAP_PROP_FPS));
    if (fps > 0) {
        if (m_videoCapture->get(cv::CAP_PROP_FPS) != fps) {
            spdlog::error("Selected fps could not be set. FPS setting now at {0}, target was {1}",
                m_videoCapture->get(cv::CAP_PROP_FPS),
                fps);
            return false;
        }
    }

    auto threadParams = WorkerThreadParams{ q, m_videoCapture.get(),
        imageStruct, m_frameNumber,
        getConfigOptions().getBool(m_configGrayscale),
        targetFps,
        m_lastTimestamp};

    m_worker.start(threadParams);
    spdlog::info("Frame aquisition with OpenCV started");
    return true;
}

void OpenCVCameraSource::stop() {
    m_worker.stop();
    if (m_videoCapture) {
        m_videoCapture->release();
    }
    m_videoCapture = nullptr;

    spdlog::info("Frame aquisition with OpenCV stopped");
}