#include "OpenVSLAMTracker.h"

#include "Utils/TimeMeasurement.h"
#include "Utils/Math.h"
#include "Utils/StringHelper.h"
#include "Utils/Transformations.h"
#include "Utils/TimeMeasurement.h"

#include <openvslam/publish/map_publisher.h>

#include <functional>
#include <sstream>

using namespace LpSlam;

OpenVSLAMTracker::OpenVSLAMTracker()
#ifdef LPSLAM_BUILD_OPENVSLAM_PANGOLIN
 :
    m_pangolinWorker( [](OpenVSLAMTracker::PangolinThreadParams params) -> bool {

    // this will only return if the user closese the gui of the viewer
    params.viewer->run();

    return false;
})
#endif
{
    //getConfigOptions().optional(m_configMarkerSize, 0.1);
}

TrackerBase::ProcessImageResult OpenVSLAMTracker::processImage(CameraQueueEntry & cam,
        std::optional<GlobalStateInTime> navResultOdom,
        std::optional<GlobalStateInTime> navResultMap,
        std::vector<SensorQueueEntry> const& sensor_data) {
    TrackerBase::ProcessImageResult res;

    std::scoped_lock slamLock(m_slamLock);    

    if (!m_slam) {
        spdlog::warn("VSLAM instance not created");
        return res;
    }

    // input the current frame and estimate the camera pose
    // timestamp is in seconds
    double imgTimestamp = 0.0001f;
    if (!m_firstImageTimestamp) {
        m_firstImageTimestamp = cam.timestamp;
    } else {
        // convert to seconds
        imgTimestamp = timestampToSeconds(m_firstImageTimestamp.value(), cam.timestamp);
    }

    SPDLOG_DEBUG("Forwarding camera image of timestamp {0} seconds to OpenVSLAM", imgTimestamp);

    const auto camPoseBefore = m_slam->get_map_publisher()->get_current_cam_pose();

    SPDLOG_DEBUG("Cam pose before frame input \n{0}", strings::streamableToString(camPoseBefore));

    
    openvslam::navigation_state nav_state;

    openvslam::Quat_t quatCamRot;
    // looking slightly up
    quatCamRot = Eigen::AngleAxis<double>((10.0 / 180.0) /3.1415,
        Eigen::Vector3d(1.0, 0.0, 0.0).normalized());

        // todo: use navigation position of this frame
    //openvslam::Mat44_t init_pose_cw = openvslam::Mat44_t::Identity();
        //init_pose_cw(0, 3) = 10.0f;
        //init_pose_cw.block<3, 3>(0, 0) = quatRotMeas.toRotationMatrix();

    openvslam::Quat_t quatNav;

    // todo: also use the map information if available
    if (navResultOdom.has_value() && m_forwardNavState) {

        //nav_state.cam_pose.block<3, 3>(0, 0) = quatRotMeas.toRotationMatrix();

        // relative camera rotation in addition to the navigation orientation
        // in space
        //openvslam::Quat_t quatNavPlusCamRot = navResult->second.orientation.value * quatCamRot;

        //cam.base.position.value
        nav_state.valid = true;

        const Quaternion qLpSlam = navResultOdom->second.orientation.value;
        // VSLAM y axis is going down, not up
        const Quaternion qVslam(qLpSlam.w(), qLpSlam.y(), -qLpSlam.x(), qLpSlam.z());

        const Vector3 pLpSlam = navResultOdom->second.position.value;
        const Vector3 pVslam(pLpSlam.y(), -pLpSlam.x(), pLpSlam.z());

        nav_state.cam_rotation = qVslam.toRotationMatrix();
        nav_state.cam_translation = pVslam;
        if (navResultOdom->second.velocityValid) {
            const Vector3 vLpSlam = navResultOdom->second.velocity.value;
            nav_state.velocity = OpenVSLAMVConvert::vectorLpSlamToOpenVSLAM(vLpSlam);
            nav_state.velocity_valid = true;
        } else {
            nav_state.velocity_valid = false;
        }
        ///nav_state.cam_pose.block<3, 3>(0, 0) = quatNavPlusCamRot.toRotationMatrix();
        //nav_state.cam_pose(0, 3) = navResult->second.position.value.x();
        //nav_state.cam_pose(1, 3) = navResult->second.position.value.y();
        //nav_state.cam_pose(2, 3) = navResult->second.position.value.z();
    } else {
        nav_state.valid = false;
    }

    //std::cout << "nav position " << cam.base.position.value << std::endl;
    SPDLOG_DEBUG("Streaming Nav State. Is Valid: {0} Rotation:\n{1}\nTranslation:\n{2}",
        nav_state.valid,
        strings::streamableToString(nav_state.cam_rotation),
        strings::streamableToString(nav_state.cam_translation));
    
    {
        TimingBase timing;
        cv::Mat zeroMask = cv::Mat{};
        m_slam->feed_monocular_frame(cam.image, imgTimestamp,zeroMask, nav_state);

        timing.end();
        SPDLOG_DEBUG("VSLAM frame processing took {0} seconds", timing.delta());
    }

    const auto camPoseAfter = m_slam->get_map_publisher()->get_current_cam_pose();
    SPDLOG_DEBUG("Cam pose after frame input \n{0}", strings::streamableToString(camPoseAfter));

    const auto frameState = m_slam->get_frame_state();
    std::stringstream trackerStatus;
    trackerStatus << "Active Keypoints " << frameState.curr_keypts << " ";
    if ( frameState.tracker_state == openvslam::tracker_state_t::NotInitialized) {
        trackerStatus << "Tracker status: NotInitialized";
    } else if ( frameState.tracker_state == openvslam::tracker_state_t::Initializing) {
        trackerStatus << "Tracker status: Initializing";
    } else if ( frameState.tracker_state == openvslam::tracker_state_t::Tracking) {
        trackerStatus << "Tracker status: Tracking";
    } else if ( frameState.tracker_state == openvslam::tracker_state_t::Lost) {
        trackerStatus << "Tracker status: Lost";
    }
    
    SPDLOG_DEBUG("VSLAM status: {0}", trackerStatus.str());

    if (frameState.tracker_state == openvslam::tracker_state_t::Tracking) {
        //exit(0);
        if (m_emitMap) {
            auto const now = std::chrono::high_resolution_clock::now();
            if ((now - m_lastEmit ) > std::chrono::seconds(5)) {
                // emit !
                emitMap(*m_sensorQ);
                m_lastEmit = now;
            }
        }

        TrackerResult tres = createTrackerResult(camPoseAfter, cam.timestamp);
        res.push_back(tres);
    }

    m_imageTracked++;

    if (m_imageTracked > 10) {
        //exit(0);
    }

    return res;
}

bool OpenVSLAMTracker::start(SensorQueue & sensorQ) {
    spdlog::info("OpenVSLAM Mono tracker starting");

    m_sensorQ = &sensorQ;
    m_lastEmit = std::chrono::high_resolution_clock::now();

    return startOpenVSlam(false);
}

bool OpenVSLAMTracker::stop() {
    return stopOpenVSlam();
}
