#include "OpenVSLAMTrackerBase.h"

#include "Utils/CustomConfig.h"
#include "Utils/FileSystem.h"
#include "InterfaceImpl/LpSlamSupport.h"

#include <openvslam/data/landmark.h>
#include <openvslam/publish/map_publisher.h>
#include <openvslam/publish/frame_publisher.h>

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

namespace LpSlam {

OpenVSLAMTrackerBase::OpenVSLAMTrackerBase() 
#ifdef LPSLAM_BUILD_OPENVSLAM_PANGOLIN
 :
    m_pangolinWorker( [](OpenVSLAMTrackerBase::PangolinThreadParams params) -> bool {

    // this will only return if the user closese the gui of the viewer
    params.viewer->run();

    return false;
})
#endif
{
    getConfigOptions().optional(m_configLiveView, false);
    getConfigOptions().optional(m_configUseMapDb, true);
    getConfigOptions().optional(m_configConfigFromFile, std::string(""));
    getConfigOptions().optional(m_configCameraSetup, std::string("monocular"));
    getConfigOptions().optional(m_configSlamKeypoints, int(1200));
    getConfigOptions().optional(m_configVocabFile, m_vocabFile);
    getConfigOptions().optional(m_configForwardNavState, true);
    getConfigOptions().optional(m_configForwardImu, true);
    getConfigOptions().optional(m_configEmitMap, false);
    getConfigOptions().optional(m_configEnableMapping, true);
    getConfigOptions().optional(m_configWaitForNavigation, false);
    getConfigOptions().optional(m_configViewerFps, int(10));
    getConfigOptions().optional(m_configforwardHighResNav, false);
    getConfigOptions().optional(m_configLoopClosure, true);
    getConfigOptions().optional(m_configUseOpenCL, false);
    getConfigOptions().optional(m_configUseCUDA, false);
    getConfigOptions().optional(m_configRelocWithNavigation, true);
    getConfigOptions().optional(m_configMapFilename, std::string("map.db"));
    getConfigOptions().optional(m_configMaxLaserAge, 1.0); // seconds
}

void OpenVSLAMTrackerBase::OnConfigurationUpdate() {
    m_useLiveView = getConfigOptions().getBool(m_configLiveView);
    m_useMapDb = getConfigOptions().getBool(m_configUseMapDb);
    m_configFromFile = getConfigOptions().getString(m_configConfigFromFile);
    m_slamKeypoints = getConfigOptions().getInteger(m_configSlamKeypoints);
    m_cameraSetup = getConfigOptions().getString(m_configCameraSetup);
    m_vocabFile = getConfigOptions().getString(m_configVocabFile);
    m_forwardNavState = getConfigOptions().getBool(m_configForwardNavState);
    m_forwardImu = getConfigOptions().getBool(m_configForwardImu);
    m_emitMap = getConfigOptions().getBool(m_configEmitMap);
    m_enableMapping = getConfigOptions().getBool(m_configEnableMapping);
    m_mapFilename = getConfigOptions().getString(m_configMapFilename);
    m_waitForNavigation = getConfigOptions().getBool(m_configWaitForNavigation);
    m_viewerFps = getConfigOptions().getInteger(m_configViewerFps);
    m_forwardHighResNav = getConfigOptions().getBool(m_configforwardHighResNav);
    m_loopClosure = getConfigOptions().getBool(m_configLoopClosure);
    m_useOpenCL = getConfigOptions().getBool(m_configUseOpenCL);
    m_useCUDA = getConfigOptions().getBool(m_configUseCUDA);
    m_relocWithNavigation = getConfigOptions().getBool(m_configRelocWithNavigation);
    m_maxLaserAge = getConfigOptions().getDouble(m_configMaxLaserAge);
}

void OpenVSLAMTrackerBase::emitMap(SensorQueue & sensorQ) {
    if (!m_slam) {
        return;
    }

    std::vector<openvslam::data::landmark*> all_landmarks;
    std::set<openvslam::data::landmark*> local_landmarks;

    m_slam->get_map_publisher()->get_landmarks(all_landmarks,
        local_landmarks);

    SPDLOG_DEBUG("Loaded {0} landmarks from OpenVSLAM", all_landmarks.size());

    FeatureList featureList;
    featureList.reserve(all_landmarks.size());

    const auto now = std::chrono::high_resolution_clock::now();

    for (auto * lm: all_landmarks) {
        // copy & paste
        featureList.emplace_back( Feature(now, now,
            OpenVSLAMVConvert::vectorOpenVSLAMToLpSlam<Position3>(lm->get_pos_in_world()),
            // todo: find closest keyframe
            OpenVSLAMVConvert::vectorOpenVSLAMToLpSlam<Position3>(lm->get_pos_in_world()),
            lm->num_observations_));
    }

    // publish
    TrackerCoordinateSystemBase base;
    sensorQ.push( SensorQueueEntry(now, base, featureList));

    SPDLOG_DEBUG("Map with {0} landmarks and anchors emitted", all_landmarks.size());
}

bool OpenVSLAMTrackerBase::startOpenVSlam(bool stereo) {

    std::shared_ptr<openvslam::config> cfg;

    if (m_configFromFile.size() > 0) {
        spdlog::info("Will load VSLAM config from file {0}", m_configFromFile);
        try {
            cfg = std::make_shared<openvslam::config>(m_configFromFile);
        } catch (const std::exception& ex) {
            std::cout << "Failed to load OpenVSLAM config from file " << m_configFromFile << std::endl;
            spdlog::error("Failed to load OpenVSLAM config file {0} because {1}",
                m_configFromFile, ex.what());
            return false;
        }
    } else {
        LpSlamCameraConfiguration primaryCam;
        if (stereo) {
            // right now, camera number hardcoded to 0 and 1
            auto [foundEntry, camConfig] = CustomConfig::getStereoCameraConfig(getCameraRegistry(), 0, 1);
            if (!foundEntry) {
                return false;
            }
            primaryCam = std::get<0>(camConfig);
        } else {
            auto [foundEntry, camConfig] = CustomConfig::getCameraConfig(getCameraRegistry(), 0);
            if (!foundEntry) {
                return false;
            }
            primaryCam = camConfig;
        }

        // populate the YAML node ourselves
        YAML::Node configNode;
        configNode["Camera.name"] = "LpSlam";
        configNode["Camera.setup"] = m_cameraSetup;

        if (primaryCam.distortion_function == LpSlamCameraDistortionFunction::LpSlamCameraDistortionFunction_Omni) {
            // scaling of fx, fy for Omni camera is hard-coded to work well with Varjo XR-3 cameras
            configNode["Camera.fx"] = primaryCam.resolution_x / 1.7;
            configNode["Camera.fy"] = primaryCam.resolution_y / 1.7;
            configNode["Camera.cx"] = primaryCam.resolution_x / 2.0;
            configNode["Camera.cy"] = primaryCam.resolution_y / 2.0;
        } else {
            configNode["Camera.fx"] = primaryCam.f_x;
            configNode["Camera.fy"] = primaryCam.f_y;
            configNode["Camera.cx"] = primaryCam.c_x;
            configNode["Camera.cy"] = primaryCam.c_y;
        }

        // for perspective camera
        // when we do the undistort in our code
        if ((primaryCam.distortion_function == LpSlamCameraDistortionFunction::LpSlamCameraDistortionFunction_Pinhole) ||
            (primaryCam.distortion_function == LpSlamCameraDistortionFunction::LpSlamCameraDistortionFunction_Omni) ||
            (primaryCam.distortion_function == LpSlamCameraDistortionFunction::LpSlamCameraDistortionFunction_NoDistortion)){
            configNode["Camera.model"] = "perspective";
            configNode["Camera.k1"] = 0.0;
            configNode["Camera.k2"] = 0.0;
            configNode["Camera.k3"] = 0.0;
            configNode["Camera.p1"] = 0.0;
            configNode["Camera.p2"] = 0.0;
        } else if (primaryCam.distortion_function == LpSlamCameraDistortionFunction::LpSlamCameraDistortionFunction_Fisheye) {
            configNode["Camera.model"] = "fisheye";
            configNode["Camera.k1"] = primaryCam.dist[0];
            configNode["Camera.k2"] = primaryCam.dist[1];
            configNode["Camera.k3"] = primaryCam.dist[2];
            configNode["Camera.k4"] = primaryCam.dist[3];
        } else {
            spdlog::error("Camera distortion not supported");
            return false;
        }

        configNode["Initializer.num_min_triangulated_pts"] = 40;
        configNode["Initializer.parallax_deg_threshold"] = 0.2; 

        configNode["Camera.fps"] = primaryCam.fps;
        configNode["Camera.cols"] = primaryCam.resolution_x;
        configNode["Camera.rows"] = primaryCam.resolution_y;

        if (stereo) {
            configNode["Camera.focal_x_baseline"] = primaryCam.focal_x_baseline;
        }
        configNode["Camera.color_order"] = "Gray";

        configNode["Feature.max_num_keypoints"] = m_slamKeypoints;
        configNode["Feature.scale_factor"] = 1.2;
        configNode["Feature.num_levels"] = 3;
        // tweaked for ZED 2
        configNode["Feature.ini_fast_threshold"] = 20;
        configNode["Feature.min_fast_threshold"] = 7;

        configNode["depth_threshold"] = 40;
        configNode["y_matching_margin"] = 2.0;

        // todo: optional only for some use cases
        configNode["wait_for_navigation_data"] = m_waitForNavigation;

        configNode["PangolinViewer.fps"] = m_viewerFps;
        configNode["use_opencl"] = m_useOpenCL;
        configNode["use_cuda"] = m_useCUDA;
        configNode["relocalize_with_nav_data"] = m_relocWithNavigation;
        configNode["time_to_relocalize"] = 3.0;

        try {
            cfg = std::make_shared<openvslam::config>(configNode);
        } catch (const std::exception& ex) {
            std::cout << "Failed to generate VSLAM config" << std::endl;
            spdlog::error("Failed to generate VSLAM config", ex.what());
            return false;
        }
    }

    // check if the vocal file is present

    if (!file_util::does_exist(m_vocabFile)) {
        spdlog::error("Vocab file {0} not present", m_vocabFile);
        return false;
    }

    // load or create camera masks
    configureMasks();

    // todo: disable for production build or make configurable
    cfg->write_logfile_ = true;

    {
        std::scoped_lock slamLock(m_slamLock);

        m_slam = std::make_unique<openvslam::system>(cfg, m_vocabFile);
        m_slam->startup(m_enableMapping);

        // check if we can load map data base
        if (m_useMapDb && file_util::does_exist(m_mapFilename)) {
            spdlog::info("Loading VSLAM map database from", m_mapFilename);
            m_slam->load_map_database(m_mapFilename);
        }

        // stop the mapping if needed
        // needs to be called after the map has been loaded
        // because map loading resumes the mapping
        if (!m_enableMapping) {
            m_slam->disable_mapping_module();
            m_slam->disable_loop_detector();
        } else if (!m_loopClosure) {
            m_slam->disable_loop_detector();
        }

        spdlog::info("OpenVSLAM tracker started");

    #ifdef LPSLAM_BUILD_OPENVSLAM_PANGOLIN
        spdlog::info("Pangolin viewer will be started {0}", m_useLiveView);
        if (m_useLiveView) {
            if (!m_viewer) {
                // pangolin can not be restarted if the user does not close the window
                m_viewer = new pangolin_viewer::viewer(cfg->yaml_node_, m_slam.get(),
                    m_slam->get_frame_publisher(), m_slam->get_map_publisher());
                m_pangolinWorker.start(PangolinThreadParams { m_viewer });
                spdlog::info("Pangolin viewer started");
            }
        }
    #else
        if (m_useLiveView) {
            spdlog::error("Not compiled with Pangolin support, can't show live view");
        }
    #endif
    }

    m_started = true;

    return true;
}

bool OpenVSLAMTrackerBase::stopOpenVSlam() {
    // do we need this request terminate call ?!
    std::scoped_lock slamLock(m_slamLock);

    if (m_slam) {
        // only save if we used the map and and mapping was enabled
        if (m_useMapDb && m_enableMapping) {
            if (m_mapFilename.size() > 0) {
                spdlog::info("Saving mapping database to {0}", m_mapFilename);
                m_slam->save_map_database(m_mapFilename);
            } else {
                spdlog::info("Won't save mapping because file name is invalid");
            }
        }

        m_slam->shutdown();
    }
    m_slam.reset();
    m_slam = nullptr;
    m_started = false;
    spdlog::info("OpenVSLAM tracker stopped");

    return true;
}

TrackerResult OpenVSLAMTrackerBase::createTrackerResult(openvslam::Mat44_t cam_pose, TimeStamp timestamp) {
    //cam_center_ = -rot_cw_.transpose() * trans_cw_;
    Eigen::Matrix3d rot_cw = cam_pose.block<3, 3>(0, 0);
    Eigen::Vector3d trans_cw = cam_pose.block<3, 1>(0, 3);
    Eigen::Vector3d cam_center = -rot_cw.transpose() * trans_cw;

    // convert to LpSlam
    Eigen::Vector3d cam_center_lpslam(-cam_center.y(), cam_center.x(), cam_center.z());
    Eigen::Quaterniond q_rot_cw_vslam(rot_cw);
    Eigen::Quaterniond rot_cw_lpslam(q_rot_cw_vslam.w(), -q_rot_cw_vslam.y(), q_rot_cw_vslam.x(), q_rot_cw_vslam.z());

    // output pose
    Eigen::Quaterniond q_orient(rot_cw_lpslam);
    Orientation orient(q_orient);

    TrackerResult tres { ResultType::TrackedVehicle,
        0,
        cam_center_lpslam,
        q_orient,
        timestamp };

    return tres;
}

bool OpenVSLAMTrackerBase::configureMasks() {

    auto lmdCreateMask = [](LpSlamCameraConfiguration const& config, bool left) {
        cv::Mat mask;

        if (config.mask_type == LpSlamCameraMaskType_Radial) {
            spdlog::info("Creating radial camera mask with radius {0}", config.mask_parameter);
            mask = cv::Mat(config.resolution_x, config.resolution_y,
                CV_8UC1, cv::Scalar(0));
            cv::circle(mask, cv::Point(config.resolution_x/2, config.resolution_y/2),
                int(config.mask_parameter),
                cv::Scalar(255), cv::FILLED, 8, 0);
        } else if (config.mask_type == LpSlamCameraMaskType_Image) {
            std::stringstream maskFileName;
            maskFileName << "camera_mask_" << (left ? "left" : "right") << ".bmp";

            spdlog::info("Loading camera mask from {0}", maskFileName.str());
            mask = cv::imread(maskFileName.str(), cv::IMREAD_GRAYSCALE);
            if (mask.empty()) {
                spdlog::error("Cannot load camera mask from file {0}", maskFileName.str());
            }
        }

        return mask;
    };

    if (getCameraRegistry() == nullptr) {
        spdlog::error("Cannot process image without camera registry");
        return false;
    } else {
        auto access = getCameraRegistry()->get();

        // configure the camera mask, if needed
        auto configLeft = access.payload().getConfiguration(0);
        if (!configLeft.has_value()) {
            spdlog::error("Cannot load configuration for camera 0");
            return false;
        }

        m_cameraMask = lmdCreateMask(configLeft.value(), true);

        // do we have a second camera?
        auto configRight = access.payload().getConfiguration(1);
        if (configRight.has_value()) {
            m_cameraMaskSecond = lmdCreateMask(configRight.value(), false);
        }
    }

    return true;
}

bool OpenVSLAMTrackerBase::mappingSetMode(bool enableMapping) {
    spdlog::info("Set VSLAM Mapping to {0}", enableMapping);
    m_enableMapping = enableMapping;
    return true;
}

bool OpenVSLAMTrackerBase::mappingSetFilename(std::string const& filename) {
    spdlog::info("Set VSLAM Mapping filename to {0}", filename);
    m_mapFilename = filename;
    return true;
}

std::size_t OpenVSLAMTrackerBase::mappingGetFeatures(LpSlamMapBoundary boundary,
    LpSlamFeatureEntry * entry, std::size_t entry_count, LpSlamMatrix9x9 transform ) {

    std::size_t copied = 0;
    auto lmd = [&copied, &entry, entry_count, transform]
        ( std::unordered_map<unsigned int, openvslam::data::landmark*> const& features ) -> void {

            Eigen::Matrix3f trans;
            trans << transform[0], transform[1], transform[2],
                transform[3], transform[4], transform[5],
                transform[6], transform[7], transform[8];

            for ( auto const& ft: features) {
                auto const worldPos = ft.second->get_pos_in_world();
                Eigen::Vector3f p_lpslam;//(-worldPos.y(), worldPos.x(), worldPos.z());
                p_lpslam << -worldPos.y(), worldPos.x(), worldPos.z();
                const Eigen::Vector3f p_transformed = trans * p_lpslam;

                entry->position = {p_transformed.x(), p_transformed.y(), p_transformed.z()};
                entry++;
                copied++;
                if (copied >= entry_count) {
                    break;
                }
            }
        };

    std::function<void(std::unordered_map<unsigned int, openvslam::data::landmark*> const&)> funcLamda
         = lmd;

    m_slam->get_map_publisher()->execute_on_landmarks(funcLamda);

    return copied;
}

std::size_t OpenVSLAMTrackerBase::mappingGetFeaturesCount(LpSlamMapBoundary boundary) {
    return m_slam->get_map_publisher()->get_landmarks_count();
}

bool OpenVSLAMTrackerBase::mappingExportCSV(std::string csv_filename) {
    return m_slam->get_map_publisher()->export_to_csv(csv_filename);
}


LpSlamStatus OpenVSLAMTrackerBase::getSlamStatus() {
    if (m_slam) {
        LpSlamStatus active = LpSlam::support::createLpSlamStatus();

        active.frame_time = m_slam->get_frame_publisher()->get_elapsed_time() * 0.001;
        active.key_frames = m_slam->get_map_publisher()->get_keyframe_count();
        active.feature_points = m_slam->get_map_publisher()->get_landmarks_count();

        return active;
    } else {
        LpSlamStatus not_active = LpSlam::support::createLpSlamStatus();
        not_active.localization = LpSlamLocalization_Off;
        return not_active;
    }
}

}
