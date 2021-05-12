#include "ZedSdkSource.h"

using namespace LpSlam;

ZedSdkCameraSource::ZedSdkCameraSource() : 
    m_worker( [](ZedSdkCameraSource::WorkerThreadParams params) -> bool {

    SPDLOG_DEBUG("Getting next frame from ZED source ...");
    // Get last available frame
    //sl::RuntimeParameters rtParams;

    if (params.videoCapture->grab() != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Cannot grab frame from ZED camera");
        // try to continue receiving, mayeb just one failed one
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return true;
    }

    sl::Mat sl_left;
    sl::Mat sl_right;
    if (params.videoCapture->retrieveImage(sl_left,sl::VIEW::LEFT_UNRECTIFIED_GRAY) != sl::ERROR_CODE::SUCCESS) { 
        spdlog::error("Got invalid frame from ZED camera");
        // try to continue receiving, mayeb just one failed one
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return true;
    }
    if (params.videoCapture->retrieveImage(sl_right,sl::VIEW::RIGHT_UNRECTIFIED_GRAY) != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Got invalid frame from ZED camera");
        // try to continue receiving, mayeb just one failed one
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return true;
    }

    cv::Mat cv_left = cv::Mat((int) sl_left.getHeight(), (int) sl_left.getWidth(), CV_8UC1, sl_left.getPtr<sl::uchar1>(sl::MEM::CPU)).clone();
    cv::Mat cv_right = cv::Mat((int) sl_right.getHeight(), (int) sl_right.getWidth(), CV_8UC1, sl_right.getPtr<sl::uchar1>(sl::MEM::CPU)).clone();


    sl_left.free(sl::MEM::CPU | sl::MEM::GPU);
    sl_right.free(sl::MEM::CPU | sl::MEM::GPU);

/*
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
*/
    params.frameNumber++;

if (params.frameNumber % 5 == 0) {
    auto mean = cv::mean(cv_left);
    auto meanVal = mean.val[0];
    float vf = meanVal / 255.0f;

    float new_gain = 30.0f + (1.0f - vf ) * 60.0f;
//spdlog::info("exp {0} {1} {2}", meanVal, vf, new_gain);
params.videoCapture->setCameraSettings(sl::VIDEO_SETTINGS::GAIN, std::round(new_gain));
}
    // for ZED its 12 cm camera distance
    TrackerCoordinateSystemBase base_left;
    TrackerCoordinateSystemBase base_right;

    // the eye line of the HMD headset is in our y-axis
    // valve index
    base_left.position.value[1] = -0.5 * 0.12;
    base_right.position.value[1] = 0.5 * 0.12;

    // todo: use hardware timestamp from camera

    auto camEntry = CameraQueueEntry( std::chrono::high_resolution_clock::now(),
        0, // left cam
        base_left,
        cv_left,
        1, // right cam
        base_right,
        cv_right
        );    

    auto sl_ttimeref = params.videoCapture->getTimestamp(sl::TIME_REFERENCE::IMAGE);

    const uint64_t ts_nsec = sl_ttimeref.getNanoseconds();
    const uint32_t sec = static_cast<uint32_t>(ts_nsec / 1000000000);
    const uint32_t nsec = static_cast<uint32_t>(ts_nsec % 1000000000);
    camEntry.ros_timestamp = LpSlamROSTimestamp{ sec, nsec};

    params.camQ.push(camEntry);

       return true; 
    }) {

}

bool ZedSdkCameraSource::start(CameraQueue & q) {
    m_cam = std::make_unique<sl::Camera>();

    // Set configuration parameters for the ZED
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    init_parameters.sdk_verbose = false;
    init_parameters.camera_fps = 15;

    auto returned_state = m_cam->open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Camera Open {0} Exit program.", returned_state);
        return false;
    }

    // working well in lightly lit room 
    m_cam->setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, 15);
    m_cam->setCameraSettings(sl::VIDEO_SETTINGS::GAIN, 50);
    //m_cam->setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, sl::VIDEO_SETTINGS_VALUE_AUTO);

    auto threadParams = WorkerThreadParams{ q, m_cam.get(),
        m_frameNumber,
        false, //useFpsScaling ? fps: 0, 
        false,
        m_lastTimestamp};

    m_worker.start(threadParams);

    return true;
}

void ZedSdkCameraSource::stop() {
    if (m_cam) {
        m_cam->close();
        m_cam.release();
    }
}
