#include "ZedOpenCaptureCameraSource.h"

#include <spdlog/spdlog.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace LpSlam;

ZedOpenCaptureCameraSource::ZedOpenCaptureCameraSource() : 
    m_worker( [](ZedOpenCaptureCameraSource::WorkerThreadParams params) -> bool {

    SPDLOG_DEBUG("Getting next frame from ZED source ...");
    // Get last available frame
    const sl_oc::video::Frame frameRaw = params.videoCapture->getLastFrame();

    if(frameRaw.data == nullptr) {
        spdlog::error("Got invalid frame from ZED camera");
        // try to continue receiving, mayeb just one failed one
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return true;
    }

    // check if we need to scale fps target because ZED camera was not
    // able to set the FPS on driver level ^^
    if (params.targetFps > 0) {
        // convert to ns
        uint64_t frameTime = 1.0 / double(params.targetFps) * std::pow(10,9);
        if ((frameRaw.timestamp - params.lastTimestamp) < frameTime) {
            // don't continue, no new frame yet
            return true;
        }
        params.lastTimestamp = frameRaw.timestamp;
    }

    params.frameNumber++;

    cv::Mat frameYUV = cv::Mat(frameRaw.height, frameRaw.width, CV_8UC2, frameRaw.data);

    // convert to grayscale before we do anything else, saves us
    // memory bandwidth later on
    cv::Mat cvFrameColorConverted;
    if (params.convertToGrayscale) {
        // converts from 2 bytes per pixel to 1 bytes per pixel
        cv::cvtColor(frameYUV, cvFrameColorConverted, cv::COLOR_YUV2GRAY_YUYV);
    } else {
        // converts from 2 bytes per pixel to 3 bytes per pixel
        cv::cvtColor(frameYUV, cvFrameColorConverted, cv::COLOR_YUV2BGR_YUYV);
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

    // todo: use hardware timestamp from camera
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

    // -1 will connect to the first available camera
    getConfigOptions().optional(m_configCameraNumber, int(-1));
    getConfigOptions().optional(m_configGrayscale, true);
    getConfigOptions().optional(m_configWidth, int(0));
    getConfigOptions().optional(m_configHeight, int(0));
    getConfigOptions().optional(m_configStereoMode, std::string(""));
    getConfigOptions().optional(m_configFps, int(0));
    getConfigOptions().optional(m_configExposure, int(0));
    getConfigOptions().optional(m_configVerbose, int(0));
    getConfigOptions().optional(m_configFpsScaling, false);
}

bool ZedOpenCaptureCameraSource::start(CameraQueue & q) {
    const auto camNum = getConfigOptions().getInteger(m_configCameraNumber);
    const auto useFpsScaling = getConfigOptions().getBool(m_configFpsScaling);
    spdlog::info("Opening ZED video capture interface number {0}", camNum);

    const auto fps = getConfigOptions().getInteger(m_configFps);
    {
        sl_oc::video::VideoParams params;
        params.verbose = getConfigOptions().getInteger(m_configVerbose);

        const auto width = getConfigOptions().getInteger(m_configWidth);
        const auto height = getConfigOptions().getInteger(m_configHeight);

        // only set the FPS if we don't use fps scaling
        // because if the FPS we want is not availble
        // opening the camera will fail
        if (!useFpsScaling) {
            if (fps == 15) {
                params.fps = sl_oc::video::FPS::FPS_15;
            } else if (fps == 30) {
                params.fps = sl_oc::video::FPS::FPS_30;
            } else if (fps == 60) {
                params.fps = sl_oc::video::FPS::FPS_60;
            } else if (fps == 100) {
                params.fps = sl_oc::video::FPS::FPS_100;
            } else if (fps == 0) {
                // just dont set anything
            } else {
                spdlog::error("FPS {0} not support by ZED camera");
                return false;
            }
        }

        if (height == 376) {
            params.res = sl_oc::video::RESOLUTION::VGA;
        } else if (height == 720) {
            params.res = sl_oc::video::RESOLUTION::HD720;
        } else if (height == 1080) {
            params.res = sl_oc::video::RESOLUTION::HD1080;
        } else if (height == 1242) {
            params.res = sl_oc::video::RESOLUTION::HD2K;
        } else if (height == 0) {
            // just don't set anything
        } else {
            spdlog::error("Resolution {0}x{1} not supported by ZED camera", width, height);
            return false;
        }

        m_videoCapture = std::make_unique<sl_oc::video::VideoCapture>(params);
        if( !m_videoCapture->initializeVideo(camNum)){
            spdlog::error("Cannot open ZED camera {0} video capture with resolution "
                "{1} x {2} and FPS {3}",             
                camNum, width, height, fps);
            return false;
        }
    }
    
    spdlog::info("ZED camera with serial number {0} opened", m_videoCapture->getSerialNumber());

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

    // note: exposure and white balance is set to auto when the
    // camera was connected
    const auto exposure = getConfigOptions().getInteger(m_configExposure);
    if (exposure > 0) {
        m_videoCapture->setExposure(sl_oc::video::CAM_SENS_POS::LEFT, exposure);
        m_videoCapture->setExposure(sl_oc::video::CAM_SENS_POS::RIGHT, exposure);
        spdlog::info("Set exposure to {0}", exposure);
    }

    {
        int configuredWidth = 0;
        int configuredHeight = 0;
        m_videoCapture->getFrameSize(configuredWidth, configuredHeight);

        spdlog::info("Selected image resolution {0} x {1}",
            configuredWidth, configuredHeight);
    }

    auto threadParams = WorkerThreadParams{ q, m_videoCapture.get(),
        imageStruct, m_frameNumber,
        useFpsScaling ? fps: 0,
        getConfigOptions().getBool(m_configGrayscale),
        m_lastTimestamp};

    m_worker.start(threadParams);
    spdlog::info("Frame aquisition with ZED camera started");
    return true;
}

void ZedOpenCaptureCameraSource::stop() {
    m_worker.stop();
    // deletes the camera object and releases the camera
    m_videoCapture.reset();

    spdlog::info("Frame aquisition with ZED camera stopped");
}