#pragma once

#include "Sources/ImageSourceBase.h"
#include "Utils/ManagedThread.h"
#include "Utils/LockedAccess.h"
#include "Utils/TimeMeasurement.h"
#include "Manager/RecordEngine.h"
#include "Manager/ReplayEngine.h"
#include "Manager/CameraRegistry.h"
#include "Processor/ProcessorBase.h"

#include "DataTypes/CameraQueue.h"
#include "DataTypes/SensorQueue.h"
#include "DataTypes/ResultQueue.h"
#include "Sources/FileImageSource.h"
#include "Interface/LpSlamTypes.h"

#include <memory>
#include <vector>
#include <mutex>
#include <optional>
#include <atomic>

namespace LpSlam {

class OpenVSLAMTrackerBase;

class SlamManager {
public:
    SlamManager();

    void start();

    void stop();

    void updateGlobalReferenceState(GlobalStateInTime const& state);

    FileImageSource * getFileSource() {
        return m_fileSource;
    }

    void addOnReconstructionCallback(OnReconstructionCallback_t callback, void * userData) {
        m_onRecoCallback = callback;
        m_onRecoCallbackUserData = userData;
    }

    void addOnImageCallback(OnImageCallback_t callback, void * userData) {
        m_onImageCallback = callback;
        m_onImageCallbackUserData = userData;
    }

    void addRequestNavDataCallback(RequestNavDataCallback_t callback, void * userData) {
        m_requestNavDataCallback = callback;
        m_requestNavDataCallbackUserData = userData;
    }

    void addRequestNavTransformationCallback(RequestNavTransformationCallback_t callback, void * userData) {
        m_requestNavTransformationCallback = callback;
        m_requestNavTransformationCallbackUserData = userData;    
    }

    void logToFile(std::string const& filename);

    bool addSource(std::string const& name, std::string const& jsonConfig);

    bool addTracker(std::string const& name, std::string const& jsonConfig);

    bool addProcessor(std::string const& name, std::string const& jsonConfig);

    bool readConfigurationFile(std::string const& filename);

    bool loadReplayItems(std::string const& filename);

    bool addImageFromBuffer(uint32_t cameraNumber, LpSlamTimestamp timestamp, uint8_t * buffer,
        LpSlamImageDescription desc);

    bool addStereoImageFromBuffer(uint32_t cameraNumber, LpSlamTimestamp timestamp, uint8_t * buffer_left,
        uint8_t * buffer_right,
        LpSlamImageDescription desc);

    std::optional<LpSlamCameraConfiguration> getCameraConfiguration(LpSlamCameraNumber num);

    void setCameraConfiguration(LpSlamCameraConfiguration const& c );

    void addMarker(LpSlamMarkerIdentifier id, LpSlamMarkerState state);

    void setLogLevel(LpSlamLogLevel level);

    void setShowLiveStream(bool b) {
        m_showLiveStream = b;
    }

    void setWriteImageFiles(bool b) {
        m_writeImageFiles = b;
    }

    void setRecord(bool b) {
        m_record = b;
    }

    // if set to false, no video images will be
    // stored in the output file of the recorder
    // only sensor data and tracking results
    void setRecordImages(bool b) {
        m_recorder.setStoreImages(b);
    }

    LpSlamStatus getSlamStatus();

    void mappingAddLaserScan(GlobalStateInTime origin,
      float * ranges, size_t rangeCount,
      float start_range,
      float end_range,
      float start_angle,
      float end_angle,
      float increment,
      float range_threshold);    

    unsigned long mappingGetMapRawSize();

    LpMapInfo mappingGetMapRaw( int8_t * map, std::size_t mapSize);

    bool mappingSetMode(bool enableMapping);

    bool mappingSetFilename(std::string const& filename);

    std::size_t mappingGetFeatures(LpSlamMapBoundary boundary,
        LpSlamFeatureEntry * entry, std::size_t entry_count, LpSlamMatrix9x9 transform );

    std::size_t mappingGetFeaturesCount(LpSlamMapBoundary boundary);

    bool mappingExportCSV(std::string csv_filename);

private:

    void setLogLevelInternal();

    std::atomic_int m_numberPic = 0;

    // if true, the occupancy map will be filled
    // how many thread to use for processing. -1 means
    // the OpenCV default will be used
    int m_thread_num = -1;
    // collects updates to the occupancy map

    typedef std::vector<std::unique_ptr<ImageSourceBase>> SourceList;
    typedef std::vector<std::unique_ptr<TrackerBase>> TrackerList;
    typedef std::vector<std::unique_ptr<ProcessorBase>> ProcessorList;

    OnReconstructionCallback_t m_onRecoCallback = nullptr;
    void * m_onRecoCallbackUserData = nullptr;

    OnImageCallback_t m_onImageCallback = nullptr;
    void * m_onImageCallbackUserData = nullptr;

    RequestNavDataCallback_t m_requestNavDataCallback = nullptr;
    void * m_requestNavDataCallbackUserData = nullptr;

    RequestNavTransformationCallback_t m_requestNavTransformationCallback = nullptr;
    void * m_requestNavTransformationCallbackUserData = nullptr;

    SourceList m_sources;
    TrackerList m_trackers;
    ProcessorList m_processors;
    LpSlamLogLevel m_logLevel = LpSlamLogLevel_Error;

    FileImageSource * m_fileSource = nullptr;

    // will be set in case an VSLAM tracker has ben added
    OpenVSLAMTrackerBase * m_vslamTracker = nullptr;

    bool m_record = false;
    bool m_showLiveStream = false;
    bool m_writeImageFiles = false;

    FramerateCompute m_framerateCompute;

    CameraRegistryAccess m_camRegistry;

    CameraQueue m_camQueue;
    SensorQueue m_sensorQueue;
    ResultQueue m_resultQueue;
    CameraQueue m_imageCallbackQueue;
    RecordEngine m_recorder;
    ReplayEngine m_replay;

    std::atomic<double> m_currentFps = 0.0;

    struct WorkerThreadParams {
        CameraQueue & camQ;
        SensorQueue & sensorQ;
        ResultQueue & resQ;
        CameraQueue & imageCallbackQ;
        TrackerList & trackerList;
        RecordEngine & recorder;
        ProcessorList & processorList;
        // todo: needs mutex
        ReplayEngine & replay;
        std::atomic_int & numberPic;
        bool showLiveStream;
        bool writeImageFiles;
        bool pushToImageCallbackQueue;
        FramerateCompute& framerateCompute;
        std::atomic<double> & currentFps;
        RequestNavDataCallback_t requestNavData_callback;
        void * requestNavData_callbackData;
    };

    ManagedThread<WorkerThreadParams> m_worker;

    struct NotifyThreadParams {
        ResultQueue & resQ;
        OnReconstructionCallback_t callback;
        void * callbackData;
    };

    ManagedThread<NotifyThreadParams> m_notifyWorker;

    struct ImageCallbackThreadParams {
        CameraQueue & camQ;
        OnImageCallback_t callback;
        void * callbackData;
    };

    ManagedThread<ImageCallbackThreadParams> m_imageCallbackWorker;

};
}
