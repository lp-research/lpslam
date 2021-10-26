#include "OpenVSLAMStereoTracker.h"

#include "Utils/TimeMeasurement.h"
#include "Utils/Math.h"
#include "Utils/StringHelper.h"
#include "Utils/Transformations.h"
#include "Utils/CustomConfig.h"
#include "InterfaceImpl/LpSlamConversion.h"

#include <openvslam/publish/map_publisher.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <future>
#include <functional>
#include <sstream>

using namespace LpSlam;

OpenVSLAMStereoTracker::OpenVSLAMStereoTracker() : OpenVSLAMTrackerBase() {
}

TrackerBase::ProcessImageResult OpenVSLAMStereoTracker::processImage(CameraQueueEntry & cam,
        std::optional<GlobalStateInTime> navResultOdom,
        std::optional<GlobalStateInTime> navResultMap,
        std::vector<SensorQueueEntry> const& sensorValues) {
    TrackerBase::ProcessImageResult res;

    std::scoped_lock slamLock(m_slamLock);
    
    if (!m_slam) {
        spdlog::warn("VSLAM instance not created");
        return res;
    }

    if (!cam.image_second.has_value()) {
        spdlog::warn("VSLAM stereo needs two images");
        return res;
    }

    // input the current frame and estimate the camera pose
    // timestamp is in seconds
    double imgTimestamp = 0.0001f;
    if (!m_firstImageTimestamp) {
        m_firstImageTimestamp = cam.timestamp;
    } else {
        auto distanceToFirst = cam.timestamp - m_firstImageTimestamp.value();
        // convert to seconds
        imgTimestamp = std::chrono::duration<double, std::chrono::seconds::period>(
            distanceToFirst
        ).count();
    }

    SPDLOG_DEBUG("Forwarding camera image of timestamp {0} seconds to OpenVSLAM", imgTimestamp);
    const auto camPoseBefore = m_slam->get_map_publisher()->get_current_cam_pose();
    SPDLOG_DEBUG("Cam pose before frame input \n{0}", strings::streamableToString(camPoseBefore));

    openvslam::Quat_t quatCamRot;
    // looking slightly up
    quatCamRot = Eigen::AngleAxis<double>((10.0 / 180.0) /3.1415,
        Eigen::Vector3d(1.0, 0.0, 0.0).normalized());

        // todo: use navigation position of this frame
    //openvslam::Mat44_t init_pose_cw = openvslam::Mat44_t::Identity();
        //init_pose_cw(0, 3) = 10.0f;
        //init_pose_cw.block<3, 3>(0, 0) = quatRotMeas.toRotationMatrix();

    // check for reference values
    int64 max_dist = std::numeric_limits<int64>::max();
    std::optional<LpSlam::GlobalState> closestRefState;
    for (auto const& sensor_meas: sensorValues) {
        if ((sensor_meas.getSensorType() == SensorQueueEntry::SensorType::GlobalState) &&
            sensor_meas.reference) {

            // find closest in time to our nav state
            LpSlam::TimeStamp navTs = navResultOdom.value().first.system_time;
            LpSlam::TimeDuration dist = navTs - sensor_meas.timestamp;
            auto dist_abs = std::abs(dist.count());
            std::cout << "dist abs " << dist_abs << std::endl;

            if (dist_abs < max_dist) {
                closestRefState = sensor_meas.getGlobalState();
                max_dist = dist_abs;
            }
        }
        std::cout << "found ref " << closestRefState.has_value() << " time diff " << max_dist << " ns" << std::endl;
    }

    if (closestRefState.has_value()) {
        const auto globalRef = closestRefState.value();
        const Quaternion qLpSlam = globalRef.orientation.value;
        // VSLAM y axis is going down, not up
        const Quaternion qVslam(qLpSlam.w(), qLpSlam.y(), -qLpSlam.x(), qLpSlam.z());

        const Vector3 pLpSlam = globalRef.position.value;
        const Vector3 pVslam(pLpSlam.y(), -pLpSlam.x(), pLpSlam.z());

        openvslam::navigation_state nav_state_ref;
        nav_state_ref.cam_rotation = qVslam.toRotationMatrix();
        nav_state_ref.cam_translation = pVslam;
        m_slam->get_map_publisher()->set_current_ref_pose(nav_state_ref);
    }

    openvslam::Quat_t quatNav;
    openvslam::navigation_state nav_state;
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
            const Vector3 vVslam(vLpSlam.y(), -vLpSlam.x(), vLpSlam.z());
            nav_state.velocity = vVslam;
            nav_state.velocity_valid = true;
        } else {
            nav_state.velocity_valid = false;
        }
    } else {
        nav_state.valid = false;
        nav_state.velocity_valid = false;
    }

    // don't continue if there is no odometry data:

    if (!nav_state.valid && m_forwardNavState) {
        spdlog::error("Skipping camera frame for tracking because no odometry information available");
        return res;
    }

    // convert also the map data for reinitiazation
    openvslam::navigation_state nav_state_map;
    if (navResultMap.has_value() && m_forwardNavState) {
        //nav_state.cam_pose.block<3, 3>(0, 0) = quatRotMeas.toRotationMatrix();

        // relative camera rotation in addition to the navigation orientation
        // in space
        //openvslam::Quat_t quatNavPlusCamRot = navResult->second.orientation.value * quatCamRot;

        //cam.base.position.value
        nav_state_map.valid = true;

        const Quaternion qLpSlam = navResultMap->second.orientation.value;
        // VSLAM y axis is going down, not up
        const Quaternion qVslam(qLpSlam.w(), qLpSlam.y(), -qLpSlam.x(), qLpSlam.z());

        const Vector3 pLpSlam = navResultMap->second.position.value;
        const Vector3 pVslam(pLpSlam.y(), -pLpSlam.x(), pLpSlam.z());

        nav_state_map.cam_rotation = qVslam.toRotationMatrix();
        nav_state_map.cam_translation = pVslam;
        if (navResultMap->second.velocityValid) {
            const Vector3 vLpSlam = navResultMap->second.velocity.value;
            const Vector3 vVslam(vLpSlam.y(), -vLpSlam.x(), vLpSlam.z());
            nav_state_map.velocity = vVslam;
            nav_state_map.velocity_valid = true;
        } else {
            nav_state_map.velocity_valid = false;
        }
    } else {
        nav_state_map.valid = false;
        nav_state_map.velocity_valid = false;
    }    

    auto camConfigPair = CustomConfig::getStereoCameraConfig(getCameraRegistry(), cam.cameraNumber,
        cam.cameraNumber_second);
    if (!camConfigPair.first) {
        // cam config not found
        spdlog::error("Cannot find camera config for camera number {0}", cam.cameraNumber);
        return res;
    }
    auto camConfig = camConfigPair.second;
    cv::Mat image_left_undist = {};
    cv::Mat image_right_undist = {};

    TimingBase timingUndist("stereo_vslam_undistort");

    // undistort camera image here, because
    // OpenVSLAM does not properly handle stereo matching
    // of fishlense images and the OpenVSlam stereo rectifier fills a big part of the image
    // with useless pixels from the very edge of the lense
    ImageProcessing::Undistort & local_undistLeft = m_undistortLeft;
    auto res_left_future = std::async([&camConfig, &cam, &image_left_undist, &local_undistLeft](){
        local_undistLeft.undistort(std::get<0>(camConfig), std::get<1>(camConfig), true, cam.image, image_left_undist);
        return true;
    });

    // shift the second camera image to the optical center of the first camera. Otherwise the stereo matching
    // won't work
    ImageProcessing::Undistort & local_undistRight = m_undistortRight;
    auto res_right_future = std::async([&camConfig, &cam, &image_right_undist, &local_undistRight](){
        local_undistRight.undistort(std::get<0>(camConfig), std::get<1>(camConfig), false, cam.image_second.value(), image_right_undist);
        return true;
    });

    res_left_future.get();
    res_right_future.get();

    timingUndist.report();

    // write out stereo corrected image
    if (false) {
        if (m_imageTracked % 15 == 0) {
            std::stringstream sleft;
            std::stringstream sright;
            sleft << "left_undistort_" << m_imageTracked << ".png";
            sright << "right_undistort_" << m_imageTracked << ".png";

            cv::imwrite(sleft.str(), image_left_undist);
            cv::imwrite(sright.str(), image_right_undist);
        }
    }

    // todo: lookup transformation between laser and camera

    openvslam::data::laser2d laser_data_to_use;
    LpSlamROSTimestamp laser_timestamp_to_use;
    // check if we have a laser measurement which is not too old
    {
        std::scoped_lock<std::mutex> laser_lock(m_laserBufferMutex);
        // do we have a laser measurement with timestamp?
        spdlog::info("m_laserBuffer.origin.first.ros_timestamp.has_value() {0}", m_laserBuffer.origin.first.ros_timestamp.has_value());
        spdlog::info("cam.ros_timestamp.has_value() {0}", cam.ros_timestamp.has_value());
        if (m_laserBuffer.origin.first.ros_timestamp.has_value() &&
            cam.ros_timestamp.has_value()) {
            const auto laser_ts = m_laserBuffer.origin.first.ros_timestamp.value();
            const auto img_ts = cam.ros_timestamp.value();
            
            const int64_t d_seconds = laser_ts.seconds - img_ts.seconds;
            const int64_t d_ns = laser_ts.nanoseconds - img_ts.nanoseconds;

            // argh, a bit fishy should be numerically safer !
            double dt = double(d_seconds) + double(d_ns) * std::pow(10.0, -9.0);

            if (std::abs(dt) < m_maxLaserAge) {
                laser_data_to_use = m_laserBuffer.laser_data;
                laser_timestamp_to_use = laser_ts;
                spdlog::info("selected laser data with dt {0} to use", dt);
            } else {
                spdlog::error("Laser data with age {0} is to old too use", dt);
            }
        } else {
            spdlog::error("Laser data has no ROS timestamp");
        }
    }

    if (laser_data_to_use.is_valid()) {
        // lookup transformation for laser data to camera!
        if (m_requestNavTransformationCallback != nullptr && cam.ros_timestamp.has_value()) {
            const auto lp_laser_to_camera = m_requestNavTransformationCallback(cam.ros_timestamp.value(),
                LpSlamNavDataFrame_Laser, LpSlamNavDataFrame_Camera,
                m_requestNavTransformationCallbackUserData);

            // Get relative movement between camera and laser at that time !
            if (lp_laser_to_camera.valid) {
                // Transform from laser -> to camera frame (or laser origin in camera's frame)
                const auto trans_lc = conversion::gsInterfaceToInternal(lp_laser_to_camera);

                // Relative translation and rotation camera -> laser
                const Vector3 t_lc = trans_lc.position.value;
                const Quaternion r_lc = trans_lc.orientation.value;

                // Update relative rotation/translation information.
                // Converting from LP to optical coordinate system.
                laser_data_to_use.trans_lc_ = openvslam::Vec3_t(t_lc.y(), -t_lc.x(), t_lc.z());
                Quaternion vslam_r_lc = Quaternion(r_lc.w(), r_lc.y(), -r_lc.x(), r_lc.z());
                laser_data_to_use.rot_lc_ = vslam_r_lc.normalized().toRotationMatrix();
            }
        }

    }

    {
        TimingBase timing("stereo_vslam_feed");
        //cv::imwrite("left.jpg", image_left_undist);
        //cv::imwrite("right.jpg", image_right_undist);
        m_slam->feed_stereo_frame(image_left_undist, image_right_undist, imgTimestamp,
            m_cameraMask,
            nav_state, nav_state_map, laser_data_to_use);
        timing.report();
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
        TrackerResult tres = createTrackerResult(camPoseAfter, cam.timestamp);
        tres.timestamp.ros_timestamp = cam.ros_timestamp;
        res.push_back(tres);
    }

    m_imageTracked++;

    if (m_imageTracked == 1) {
        //exit(0);
        //std::this_thread::sleep_for(std::chrono::seconds(30));
    }

    return res;
}

bool OpenVSLAMStereoTracker::start(SensorQueue &) {
    spdlog::info("OpenVSLAM Stereo tracker starting");

    return startOpenVSlam();
}

bool OpenVSLAMStereoTracker::stop() {
    spdlog::info("OpenVSLAM Stereo tracker stopping");
    return stopOpenVSlam();
}

void OpenVSLAMStereoTracker::addLaserScan(GlobalStateInTime origin,
    float * ranges, size_t rangeCount,
    float start_range,
    float end_range,
    float start_angle,
    float end_angle,
    float increment,
    float range_threshold) {

    // buffer to send together with the next camera image
    std::scoped_lock<std::mutex> laserLock(m_laserBufferMutex);

    m_laserBuffer.laser_data.ranges_.resize(rangeCount);
    spdlog::info("Copied {0} laser data has timestamp {1}", rangeCount, origin.first.ros_timestamp.has_value());
    std::memcpy(m_laserBuffer.laser_data.ranges_.data(), ranges, sizeof(float) * rangeCount);
    m_laserBuffer.origin = origin;
    //! start angle of the scan [rad]
    m_laserBuffer.laser_data.angle_min_ = start_angle;
    //! end angle of the scan [rad]
    m_laserBuffer.laser_data.angle_max_ = end_angle;
    //! 200 measurements over the rangen [rad]
    m_laserBuffer.laser_data.angle_increment_ = increment;

    //! minimum range value [m]
    m_laserBuffer.laser_data.range_min_ = start_range;
    //! maximum range value [m]
    m_laserBuffer.laser_data.range_max_ = end_range;
}


std::optional<unsigned long> OpenVSLAMStereoTracker::mappingGetMapRawSize() {
    if (m_slam == nullptr) {
        return std::nullopt;
    }

    return m_slam->get_map_publisher()->occupancy_map_export_size_required();
}

std::optional<LpMapInfo> OpenVSLAMStereoTracker::mappingGetMapRaw(int8_t * map, std::size_t mapSize) {
    if (m_slam == nullptr) {
        return std::nullopt;
    }

    TimingBase timing("occ export");
    auto vslam_info = m_slam->get_map_publisher()->occupancy_map_export(map, mapSize);
    spdlog::info("Occupancy map generation took {0} ms", timing.end() * 1000.0);

    LpMapInfo lp_info;
    lp_info.x_cell_count = vslam_info.width;
    lp_info.y_cell_count = vslam_info.height;

    lp_info.x_cell_size = vslam_info.resolution;
    lp_info.y_cell_size = vslam_info.resolution;

    lp_info.x_origin = vslam_info.origin_x;
    lp_info.y_origin = vslam_info.origin_y;

    return lp_info;
}