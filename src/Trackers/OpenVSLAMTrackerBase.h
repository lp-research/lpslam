#pragma once

#include "DataTypes/Space.h"
#include "Trackers/TrackerBase.h"
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
#include <mutex>

namespace LpSlam {

namespace OpenVSLAMVConvert{

    template <class TPos>
    inline auto vectorLpSlamToOpenVSLAM (TPos lp) {
        return openvslam::Vec3_t(lp.y(), -lp.x(), lp.z());
    }

    template <class TPos>
    inline auto vectorOpenVSLAMToLpSlam (openvslam::Vec3_t ov) {
        return TPos(-ov.y(), ov.x(), ov.z());
    }
}

class OpenVSLAMTrackerBase : public TrackerBase {
public:
    virtual ~OpenVSLAMTrackerBase() = default;

    OpenVSLAMTrackerBase();
    virtual void OnConfigurationUpdate() override;
    bool startOpenVSlam(bool stereo = true);
    bool stopOpenVSlam();

    bool mappingSetMode(bool enableMapping);

    bool mappingSetFilename(std::string const& filename);

    bool mappingExportCSV(std::string csv_filename);

    LpSlamStatus getSlamStatus();

    std::size_t mappingGetFeatures(LpSlamMapBoundary boundary,
        LpSlamFeatureEntry * entry, std::size_t entry_count, LpSlamMatrix9x9 transform );

    std::size_t mappingGetFeaturesCount(LpSlamMapBoundary boundary);

    void addRequestNavTransformationCallback(RequestNavTransformationCallback_t callback, void * userData) override {
        m_requestNavTransformationCallback = callback;
        m_requestNavTransformationCallbackUserData = userData;    
    }

private:
    bool configureMasks();

protected:

    RequestNavTransformationCallback_t m_requestNavTransformationCallback = nullptr;
    void * m_requestNavTransformationCallbackUserData = nullptr;


    bool m_useLiveView = false;
    bool m_useMapDb = true;
    bool m_forwardNavState = true;
    bool m_forwardImu = true;
    bool m_emitMap = false;
    bool m_enableMapping = true;
    bool m_waitForNavigation = false;
    std::atomic<bool> m_forwardHighResNav = false;
    std::atomic<bool> m_started = false;
    bool m_loopClosure = true;
    bool m_useOpenCL = false;
    bool m_useCUDA = false;
    bool m_relocWithNavigation = true;
    std::string m_configFromFile = "";
    std::string m_mapFilename;
    std::string m_cameraSetup = "monocular";
    std::string m_vocabFile = "orb_vocab.dbow2";
    int m_slamKeypoints;
    int m_viewerFps;    
    float m_maxLaserAge;
    std::unique_ptr<openvslam::system> m_slam;
    std::mutex m_slamLock;

    // caller needs to hold lock on m_slamLock
    void emitMap(SensorQueue & sensorQ);

    TrackerResult createTrackerResult(openvslam::Mat44_t cam_pose, TimeStamp timestamp);

    // for left of mono camera
    cv::Mat m_cameraMask;
    // only used for stereo camera (then its the right cam)
    cv::Mat m_cameraMaskSecond;

private:
    const std::string m_configLiveView = "liveView";
    const std::string m_configUseMapDb = "useMapDb";
    const std::string m_configConfigFromFile = "configFromFile";
    const std::string m_configSlamKeypoints = "slamKeypoints";
    const std::string m_configVocabFile = "vocabFile";
    const std::string m_configCameraSetup = "cameraSetup";
    const std::string m_configForwardNavState = "forwardNavState";
    const std::string m_configForwardImu = "forwardImu";
    const std::string m_configEmitMap = "emitMap";
    const std::string m_configLoopClosure = "loopClosure";
    const std::string m_configEnableMapping = "enableMapping";
    const std::string m_configMapFilename = "mapFilename";
    const std::string m_configWaitForNavigation = "waitForNavigation";
    const std::string m_configRelocWithNavigation = "relocWithNavigation";
    const std::string m_configViewerFps = "viewerFps";
    const std::string m_configMaxLaserAge = "maxLaserAge";
    // forward navigation datat with a higher rate than the image fps
    // mostly useful for smoother visualization
    const std::string m_configforwardHighResNav = "forwardHighResNav";
    const std::string m_configUseOpenCL = "useOpenCL";
    const std::string m_configUseCUDA = "useCUDA";

#ifdef LPSLAM_BUILD_OPENVSLAM_PANGOLIN
    //std::unique_ptr<pangolin_viewer::viewer> m_viewer;
    pangolin_viewer::viewer* m_viewer = nullptr;

    struct PangolinThreadParams
    {
        pangolin_viewer::viewer * viewer;
    };

    ManagedThread<PangolinThreadParams> m_pangolinWorker;
#endif

};

}