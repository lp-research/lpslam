#include "SlamManager.h"

#include "Sources/OpenCVCameraSource.h"

#ifdef LPSLAM_BUILD_ZEDCAM
#include "Sources/ZedOpenCaptureCameraSource.h"
#endif

#ifdef LPSLAM_BUILD_OPENVSLAM
#include "Trackers/OpenVSLAMTracker.h"
#include "Trackers/OpenVSLAMStereoTracker.h"
#endif

#ifdef LPSLAM_BUILD_WEBOTS
#include "Sources/WebotsSource.h"
#endif

#ifdef LPSLAM_BUILD_ZEDSDK
#include "Sources/ZedSdkSource.h"
#endif

#include "Utils/Transformations.h"
#include "Utils/StringHelper.h"
#include "Interface/LpSlamConfiguration.h"
#include "InterfaceImpl/LpSlamConversion.h"
#include "InterfaceImpl/LpSlamSupport.h"

#include "Processor/BlackoutImageProcessor.h"
#include "Processor/AdjustIntensityProcessor.h"
#include "Processor/CameraCalibrationProcessor.h"

#include <nlohmann/json.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>

#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>


using namespace LpSlam;

SlamManager::SlamManager() :
    m_replay(m_camQueue, m_sensorQueue),
    m_worker( [](SlamManager::WorkerThreadParams params) -> bool {

        // check if we need to stream in more replay items
        params.replay.streamMoreReplayItems();

        // wait for camera
        CameraQueueEntry camEntry;
        params.camQ.pop(camEntry);

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (params.pushToImageCallbackQueue) {
            params.imageCallbackQ.push(camEntry);
        }

        // sign to exit
        if (camEntry.valid == false) {
            return false;
        }

        if (params.writeImageFiles) {
            // raw dumping of image for testing
            std::stringstream sFileOut, sFileOutLower;
            sFileOut << params.numberPic << "_left.jpg";
            sFileOutLower << params.numberPic << "_right.jpg";
            if ((params.numberPic % 10) == 0) {
                cv::imwrite(sFileOut.str(), camEntry.image);
                if ( camEntry.image_second.has_value()) {
                    cv::imwrite(sFileOutLower.str(), camEntry.image_second.value());
                }
            }
        }
        params.numberPic++;

        spdlog::debug("popped camera image of size {0}x{1}, {2} left in queue",
            camEntry.image.cols, camEntry.image.rows,
            params.camQ.size());

        params.framerateCompute.gotFrame();
        if ((params.numberPic % 30) == 0) {
            auto fps = params.framerateCompute.getFramerate();
            if (fps.has_value()) {
                params.currentFps = fps.value();
                spdlog::info("Camera images coming in with {0} fps", fps.value());
            }
        }

        // get all sensor values up to this image !
        SensorQueueEntry sensorEntry;
        std::vector<SensorQueueEntry> thisSensorEntries;
        while (params.sensorQ.try_pop(sensorEntry)) {
            params.recorder.storeSensor(sensorEntry);

            thisSensorEntries.push_back(sensorEntry);
            // only get sensor values up to the current camera image
            if (sensorEntry.timestamp > camEntry.timestamp) {
                break;
            }
        }

        // execute processors, do this after the files have been saved to disk
        for (auto & processor: params.processorList) {
            processor->processImage(camEntry);
        }

        if (params.showLiveStream) {
            // the image show method can be very slow with greyscale images for some reason .. ^^
            if ((params.numberPic % 10) == 0) {
                cv::namedWindow( "LpSlam Live (left)", cv::WINDOW_AUTOSIZE );
                cv::imshow("LpSlam Live (left)", camEntry.image );

                if ( camEntry.image_second.has_value()) {
                    cv::namedWindow("LpSlam Live (right)", cv::WINDOW_AUTOSIZE );
                    cv::imshow("LpSlam Live (right)", camEntry.image_second.value() );
                }

                cv::waitKey(25);
            }
        }

        // could very well be nullptr
        std::optional<GlobalStateInTime> latestVehicleStateOdom = std::nullopt;
        std::optional<GlobalStateInTime> latestVehicleStateMap = std::nullopt;

        // these states are at the same time as the camera image so we can use the same timestamp
        /*
        not working when the robot is acutally used
        if (camEntry.globalState.has_value()) {
            latestVehicleStateOdom = { CompositeTimestamp(camEntry.timestamp), camEntry.globalState.value() };
        }
        if (camEntry.globalState_map.has_value()) {
            latestVehicleStateMap = { CompositeTimestamp(camEntry.timestamp), camEntry.globalState_map.value() };
        }
        */
        // should we get nav data for that via a callback ?
        if ((params.requestNavData_callback != nullptr) && (camEntry.ros_timestamp.has_value())) {
            LpSlamGlobalStateInTime lpOdomState;
            LpSlamGlobalStateInTime lpMapState;

            TimingBase navRequestTiming;
            auto res = params.requestNavData_callback(camEntry.ros_timestamp.value(), 
                &lpOdomState,
                &lpMapState,
                params.requestNavData_callbackData);
            
            spdlog::info("requestin nav data took {0} s", navRequestTiming.end());

            if ( ( res == LpSlamRequestNavDataResult_OdomOnly) || 
                ( res == LpSlamRequestNavDataResult_OdomAndMap)) {
                // succcesful
                auto odom_state = conversion::gsInTimeInterfaceToInternal(lpOdomState);
                latestVehicleStateOdom = odom_state;
            }

            if ( ( res == LpSlamRequestNavDataResult_MapOnly) || 
                ( res == LpSlamRequestNavDataResult_OdomAndMap)) {
                // succcesful
                auto map_state = conversion::gsInTimeInterfaceToInternal(lpMapState);
                latestVehicleStateMap = map_state;
            }

            // handle if we could not lookup something
            // have no odometry ?
            if ( ( res == LpSlamRequestNavDataResult_MapOnly) ||
                ( res == LpSlamRequestNavDataResult_None)) {
                latestVehicleStateOdom = std::nullopt;
            }
            // have no map ?
            if ( ( res == LpSlamRequestNavDataResult_OdomOnly) ||
                ( res == LpSlamRequestNavDataResult_None)) {
                latestVehicleStateMap = std::nullopt;
            }
        }

        params.recorder.storeCameraImage(camEntry, latestVehicleStateOdom, latestVehicleStateMap);

        bool resultSent = false;
        // foward to predictions
        for (auto & tracker: params.trackerList) {
            
            if (!latestVehicleStateOdom.has_value()) {
                spdlog::warn("No vehicle odometry, skipping frame");
                break;
            }

            TimingBase timingProcess;
            auto trackingResults = tracker->processImage(camEntry, latestVehicleStateOdom,
                latestVehicleStateMap,
                thisSensorEntries);
            spdlog::debug("Running tracker took {0} seconds", timingProcess.end());

            spdlog::debug("Found {0} tracked object in image", trackingResults.size());

            // handle all tracking results provided
            for (auto & trackingResult: trackingResults) {

                spdlog::info("Processing tracker result id {0} position {1} orientation {2}",
                    trackingResult.id, strings::positionToString(trackingResult.position),
                    strings::orientationToString(trackingResult.orientation));

                {
                    const GlobalState gs(trackingResult.position, trackingResult.orientation);
                    const GlobalStateInTime newState = {trackingResult.timestamp, gs};
                    params.recorder.storeResult(newState);

                    // execute processors on results an sensor data
                    for (auto & processor: params.processorList) {
                        processor->processSensorValuesAndResults(thisSensorEntries,
                            newState);
                    }

                    params.resQ.push(newState);
                    resultSent = true;
                }
            }
        }
        
        if (!resultSent) {
            // sent at least one invalid reconstructon so the clients know we have processed
            // the image
            GlobalStateInTime gs_time;
            gs_time.second.stateValid = false;
            params.resQ.push(gs_time);
        }

        return true;
    }),
    m_notifyWorker([](SlamManager::NotifyThreadParams params) -> bool {

        ResultQueueEntry resultEntry;

        params.resQ.pop(resultEntry);
        if (resultEntry.exitSignal) {
            return false;
        }

        // send off to callback function
        if (params.callback != nullptr) {
            params.callback(conversion::gsInTimeInternalToInterface(resultEntry.globalStateInTime),
                params.callbackData);
        }

        // continue and wait for next result
        return true;
     }),
    m_imageCallbackWorker([](SlamManager::ImageCallbackThreadParams params) -> bool {

        CameraQueueEntry qEntry;

        params.camQ.pop(qEntry);
        if (qEntry.valid == false) {
            return false;
        }

        // send off to callback function
        if ((params.callback != nullptr) && (!qEntry.image.empty())) {
            // todo: only supports mono cam so far
            std::vector<unsigned char> imgData;
            std::vector<unsigned char> imgData_right;

            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(70); //image quality

            std::thread t_left([&imgData, &qEntry, compression_params](){
                cv::imencode(".jpg", qEntry.image, imgData, compression_params);
            });

            if (qEntry.image_second.has_value()) {
                // append the second image to the buffer
                std::thread t_right([&imgData_right, &qEntry, compression_params](){
                    cv::imencode(".jpg", qEntry.image_second.value(), imgData_right, compression_params);
                });

                t_right.join();
            }
            t_left.join();

            LpSlamImageDescription desc;
            // mono camera image
            desc.imageSize = imgData.size();
            desc.imageSizeSecond = 0;
            desc.image_conversion = LpSlamImageConversion::LpSlamImageConversion_None;
            desc.structure = LpSlamImageStructure_OneImage_Compressed;
            desc.format = LpSlamImageFormat::LpSlamImageFormat_8UC1_JPEPG;

            if (qEntry.image_second.has_value()) {
                desc.imageSizeSecond = imgData_right.size();
                desc.structure = LpSlamImageStructure_Stereo_Compressed;

                // copy to main buffer
                imgData.insert(std::end(imgData),
                    std::begin(imgData_right), std::end(imgData_right));
            }

            params.callback(qEntry.timestamp.time_since_epoch().count(),
                qEntry.cameraNumber, imgData.data(), desc, params.callbackData);
        }

        // continue and wait for next result
        return true;
     } )
    {

    // check if our logger exists and only reconfigure
    // if not done yet.
    std::string lpslam_logger_name = "lpslam_console";
    if (spdlog::get(lpslam_logger_name) == nullptr) {
        auto console = spdlog::stdout_logger_mt(lpslam_logger_name);
        // don't put any changing numbers in the output
        spdlog::set_pattern("*** %v ***");
        spdlog::set_level(spdlog::level::warn);
    }

    setLogLevelInternal();
}

void SlamManager::mappingAddLaserScan(GlobalStateInTime origin,
    float * ranges, size_t rangeCount,
    float start_range,
    float end_range,
    float start_angle,
    float end_angle,
    float increment,
    float range_threshold) {

    // add laser scan also to OpenVSLAM
    // notify trackers, in case they need it earlier
    for (auto & tracker: m_trackers) {
        tracker->addLaserScan(origin,
            ranges, rangeCount,
            start_range,
            end_range,
            start_angle,
            end_angle,
            increment,
            range_threshold);
    }
}

void SlamManager::updateGlobalReferenceState(GlobalStateInTime const& state) {
    // just put on the sensor queue, we treat it like sensor measurements
    TrackerCoordinateSystemBase b;
    SensorQueueEntry qEntry(state.first.system_time, b, state.second);
    qEntry.reference = true;
    m_sensorQueue.push(qEntry);
}

std::optional<LpSlamCameraConfiguration> SlamManager::getCameraConfiguration(LpSlamCameraNumber num) {
    auto access = m_camRegistry.get();
    return access.payload().getConfiguration(num);
}

void SlamManager::setCameraConfiguration(LpSlamCameraConfiguration const& c ) {
    auto access = m_camRegistry.get();
    return access.payload().setConfiguration(c);
}

unsigned long SlamManager::mappingGetMapRawSize() {
    for (auto & tracker: m_trackers) {
        auto res = tracker->mappingGetMapRawSize();
        if (res.has_value()) {
            return res.value();
        }
    }

    return 0;
}

LpMapInfo SlamManager::mappingGetMapRaw( int8_t * map, std::size_t mapSize) {
    for (auto & tracker: m_trackers) {
        auto res = tracker->mappingGetMapRaw(map, mapSize);
        if (res.has_value()) {
            return res.value();
        }
    }

    return LpMapInfo();
}

bool SlamManager::addProcessor(std::string const& name, std::string const& jsonConfig) {
    try {
        std::unique_ptr<ProcessorBase> processor;
        if (name == BlackoutImageProcessor().type()) {
            processor = std::make_unique<BlackoutImageProcessor>();
        }
        if (name == AdjustIntensityProcessor().type()) {
            processor = std::make_unique<AdjustIntensityProcessor>();
        }
        if (name == CameraCalibrationProcessor().type()) {
            processor = std::make_unique<CameraCalibrationProcessor>();
        }

        if (processor) {
            processor->setConfig(jsonConfig);
            m_processors.emplace_back(std::move(processor));
            spdlog::info("Added processor with name {0}", name);
            return true;
        }
    }
    catch (std::invalid_argument & err) {
        spdlog::error("Cannot add processor of type {0} because: {1}",
            name, err.what());
        return false;
    }

    spdlog::error("Processor with name {0} not found", name);
    return false;
}

bool SlamManager::addSource(std::string const& name, std::string const& jsonConfig) {
    try {
        std::unique_ptr<ImageSourceBase> source;
        #ifdef LPSLAM_BUILD_WEBOTS
        if (name == "Webots") {
            source = std::make_unique<WebotsSource>();
        }
        #endif
        #ifdef LPSLAM_BUILD_ZEDCAM
        if (name == "Zed") {
            source = std::make_unique<ZedOpenCaptureCameraSource>();
        }
        #endif
        #ifdef LPSLAM_BUILD_ZEDSDK
        if (name == "ZedSdk") {
            source = std::make_unique<ZedSdkCameraSource>();
        }
        #endif

        if (name == "FileSource") {
            source = std::make_unique<FileImageSource>();
            m_fileSource = static_cast<FileImageSource *>(source.get());
        }

        if (name == "OpenCV") {
            source = std::make_unique<OpenCVCameraSource>();
        }

        if (source) {
            source->setCameraRegistry(&m_camRegistry);
            source->setConfig(jsonConfig);
            m_sources.emplace_back(std::move(source));
            return true;
        }
    }
    catch (std::invalid_argument & err) {
        spdlog::error("Cannot add datasource of type {0} because: {1}",
            name, err.what());
        return false;
    }

    spdlog::error("Source with name {0} not found", name);
    return false;
}

bool SlamManager::addTracker(std::string const& name, std::string const& jsonConfig) {
    try {
        std::unique_ptr<TrackerBase> tracker;
        #ifdef LPSLAM_BUILD_OPENVSLAM
        if (name == "VSLAMStereo") {
            tracker = std::make_unique<OpenVSLAMStereoTracker>();
            m_vslamTracker = static_cast<OpenVSLAMTrackerBase*>(tracker.get());
        }
        if (name == "VSLAMMono") {
            tracker = std::make_unique<OpenVSLAMTracker>();
            m_vslamTracker = static_cast<OpenVSLAMTrackerBase*>(tracker.get());
        }
        #endif

        if (tracker) {
            if (!tracker->setConfig(jsonConfig)) {
                spdlog::error("Cannot parse config for tracker", tracker->type());
                return false;
            }
            tracker->setCameraRegistry(&m_camRegistry);
            spdlog::info("Tracker {0} added", tracker->type());
            m_trackers.emplace_back(std::move(tracker));
            return true;
        }
    }
    catch (std::invalid_argument & err) {
        spdlog::error("Cannot add tracker of type {0} because: {1}",
            name, err.what());
        return false;
    }

    spdlog::error("Tracker with name {0} not found", name);
    return false;
}

bool SlamManager::loadReplayItems(std::string const& filename) {
    return m_replay.loadReplayItems(filename);
}

void SlamManager::start() {

    std::cout << "Starting SlamManager with " << m_trackers.size()
        << " trackers and " << m_sources.size() << " sources" << std::endl;

    if (m_thread_num > -1) {
        spdlog::info("Setting thread number to {0}", m_thread_num);
        cv::setNumThreads(m_thread_num);
    }

    for (auto & tracker : m_trackers) {
        tracker->start(m_sensorQueue);

        // update callbacks
        tracker->addRequestNavTransformationCallback(m_requestNavTransformationCallback,
            m_requestNavTransformationCallbackUserData);
    }

    for (auto & src : m_sources) {
        // some sources rely on the sensors being started first !
        src-> startSensor(m_sensorQueue);
        src-> start(m_camQueue);
    }

    // wait for image
    auto workerParams = WorkerThreadParams {
        m_camQueue,
        m_sensorQueue,
        m_resultQueue,
        m_imageCallbackQueue,
        m_trackers,
        m_recorder,
        m_processors,
        m_replay,
        m_numberPic,
        m_showLiveStream,
        m_writeImageFiles,
        m_onRecoCallback != nullptr,
        m_framerateCompute,
        m_currentFps,
        m_requestNavDataCallback,
        m_requestNavDataCallbackUserData};
    m_worker.start(workerParams);

    auto notifyParams = NotifyThreadParams {
        m_resultQueue,
        m_onRecoCallback,
        m_onRecoCallbackUserData
    };
    m_notifyWorker.start(notifyParams);

    auto imageCallbackQueue = ImageCallbackThreadParams {
        m_imageCallbackQueue,
        m_onImageCallback,
        m_onImageCallbackUserData
    };
    m_imageCallbackWorker.start(imageCallbackQueue);

    if (m_record) {
        std::time_t timeNow = std::time(nullptr);
        std::tm tm = *std::localtime(&timeNow);
        std::stringstream sDate;
        sDate << "slam_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".pb";

        m_recorder.setOutputFile(sDate.str());
        m_recorder.start();
    }

    spdlog::info("Processing started with {0} data sources and {1} tracker",
        m_sources.size(), m_trackers.size());
}

void SlamManager::stop() {

    m_worker.stopAsync();

    CameraQueueEntry exitEntry;
    exitEntry.valid = false;
    m_camQueue.push(exitEntry);
    m_worker.stop();

    m_imageCallbackQueue.push(exitEntry);
    m_imageCallbackWorker.stop();

    for (auto & src : m_sources) {
        src-> stop();
    }

    for (auto & tracker : m_trackers) {
        tracker->stop();
    }

    m_camQueue.clear();

    m_notifyWorker.stopAsync();

    ResultQueueEntry exitNotifyThread;
    exitNotifyThread.exitSignal = true;
    m_resultQueue.push(exitNotifyThread);
    m_notifyWorker.stop();
    m_resultQueue.clear();
    m_recorder.stop();

    spdlog::info("Processing stopped");
}

bool SlamManager::readConfigurationFile(std::string const& filename) {
    // load the data sources from a config file
    nlohmann::json j_in;

    spdlog::info("Loading configuration from {0}", filename);

    try {
        std::ifstream ifs(filename, std::ifstream::in);

        if (!ifs) {
            spdlog::error("Cannot open config file {0}", filename);
            return false;
        }

        try {
            j_in = nlohmann::json::parse(ifs);
        }
        catch (nlohmann::json::parse_error &err) {
            spdlog::error("Cannot parse config file {0} because: {1}", filename,
                err.what());
            return false;
        }
    }
    catch (std::ifstream::failure & e) {
        spdlog::error("Cannot read config file {0} because: {1}", filename,
            e.what());
        return false;
    }

    m_recorder.setWriteRawFile(false);

    auto j_manager = j_in.find("manager");

    if (j_manager != j_in.end()) {
        auto j_record = j_manager->find("record");
        if (j_record != j_manager->end()) {
            m_record = j_record.value().get<bool>();
        }

        auto j_thread_num = j_manager->find("thread_num");
        if (j_thread_num != j_manager->end()) {
            m_thread_num = j_thread_num.value().get<int>();
        }

        auto j_show_live = j_manager->find("show_live");
        if (j_show_live != j_manager->end()) {
            m_showLiveStream = j_show_live.value().get<bool>();
        }

        auto j_record_raw = j_manager->find("record_raw");
        if (j_record_raw != j_manager->end()) {
            bool record_raw = j_record_raw.value().get<bool>();
            m_recorder.setWriteRawFile(record_raw);
        }

        auto j_replay_chunk = j_manager->find("replay_chunks");
        if (j_replay_chunk != j_manager->end()) {
            size_t chunk = j_replay_chunk.value().get<size_t>();
            m_replay.setLoadImageChunk(chunk);
        }
    }

    auto j_trackers = j_in.find("trackers");

    if (j_trackers != j_in.end()) {
        auto j_tracker_list = j_trackers.value();

        // should be a list
        for (auto j_tracker : j_tracker_list) {
            auto j_type = j_tracker.find("type");
            auto j_type_ignore = j_tracker.find("_type");
            auto j_config = j_tracker.find("configuration");

            if (j_type_ignore != j_tracker.end()) {
                continue;
            }

            if (j_type == j_tracker.end()) {
                spdlog::error("Tracker entry is missing required field 'type'");
                return false;
            }

            std::string configJson;
            if (j_config != j_tracker.end()) {
                configJson = j_config.value().dump();
            } else {
                spdlog::warn("Tracker entry {0} has no 'configuration' field. Will use default configuration.",
                    filename);
            }

            const auto trackerName = j_type.value().get<std::string>();
            if (!addTracker(trackerName, configJson)) {
                spdlog::error("Adding tracker of type {0} with config:\n {1} \n failed",
                    trackerName, configJson);
                return false;
            }
        }
    } else {
        spdlog::warn("Config file {0} contains no configured trackers", filename);
    }

    auto j_processors = j_in.find("processors");

    if (j_processors != j_in.end()) {
        auto j_processor_list = j_processors.value();

        // should be a list
        for (auto j_processor : j_processor_list) {
            auto j_type = j_processor.find("type");
            auto j_type_ignore = j_processor.find("_type");
            auto j_config = j_processor.find("configuration");

            if (j_type_ignore != j_processor.end()) {
                continue;
            }

            if (j_type == j_processor.end()) {
                spdlog::error("Processor entry is missing required field 'type'");
                return false;
            }

            std::string configJson;
            if (j_config != j_processor.end()) {
                configJson = j_config.value().dump();
            } else {
                spdlog::warn("Processor entry {0} has no 'configuration' field. Will use default configuration.",
                    filename);
            }

            const auto processorName = j_type.value().get<std::string>();
            if (!addProcessor(processorName, configJson)) {
                spdlog::error("Adding processor of type {0} with config:\n {1} \n failed",
                    processorName, configJson);
                return false;
            }
        }
    }

    auto j_datasources = j_in.find("datasources");

    if (j_datasources != j_in.end()) {
        auto j_datasource_list = j_datasources.value();

        // should be a list
        for (auto j_datasource : j_datasource_list) {
            auto j_type = j_datasource.find("type");
            auto j_type_ignore = j_datasource.find("_type");
            auto j_config = j_datasource.find("configuration");

            if (j_type_ignore != j_datasource.end()) {
                continue;
            }

            if (j_type == j_datasource.end()) {
                spdlog::error("Data source entry is missing required field 'type'");
                return false;
            }

            std::string configJson;
            if (j_config != j_datasource.end()) {
                configJson = j_config.value().dump();
            } else {
                spdlog::warn("Data store entry {0} has no 'configuration' field. Will use default configuration.",
                    filename);
            }

            const auto dataSourceName = j_type.value().get<std::string>();
            if (!addSource(dataSourceName, configJson)) {
                spdlog::error("Adding datasource of type {0} with config:\n {1} \n failed",
                    dataSourceName, configJson);
                return false;
            }

        }
    } else {
        spdlog::warn("Config file {0} contains no configured data sources", filename);
    }


    auto j_cameras = j_in.find("cameras");

    if (j_cameras != j_in.end()) {
        auto j_camera_list = j_cameras.value();

        // should be a list
        for (auto j_camera : j_camera_list) {
            auto j_model = j_camera.find("model");
            auto j_number = j_camera.find("number");
            if (j_model == j_camera.end()) {
                spdlog::error("Camera entry is missing required field 'model'");
                return false;
            }
            if (j_number == j_camera.end()) {
                spdlog::error("Camera entry is missing required field 'number'");
                return false;
            }

            auto j_fx = j_camera.find("fx");
            auto j_fy = j_camera.find("fy");
            auto j_cx = j_camera.find("cx");
            auto j_cy = j_camera.find("cy");

            if (j_fx == j_camera.end()) {
                spdlog::error("Camera entry is missing required field 'fx'");
                return false;
            }
            if (j_fy == j_camera.end()) {
                spdlog::error("Camera entry is missing required field 'fy'");
                return false;
            }
            if (j_cx == j_camera.end()) {
                spdlog::error("Camera entry is missing required field 'cx'");
                return false;
            }
            if (j_cx == j_camera.end()) {
                spdlog::error("Camera entry is missing required field 'cy'");
                return false;
            }

            auto j_resolution_x = j_camera.find("resolution_x");
            if (j_resolution_x == j_camera.end()) {
                spdlog::error("Camera entry is missing required list 'resolution_x'");
                return false;
            }

            auto j_resolution_y = j_camera.find("resolution_y");
            if (j_resolution_y == j_camera.end()) {
                spdlog::error("Camera entry is missing required list 'resolution_y'");
                return false;
            }

            auto j_rotation = j_camera.find("rotation");
            auto j_translation = j_camera.find("translation");
            // rotation in vector form
            auto j_rotation_vec = j_camera.find("rotation_vec");

            auto j_fps = j_camera.find("fps");
            auto j_focal_x_baseline = j_camera.find("focal_x_baseline");

            LpSlamCameraConfiguration cam_config = LpSlamConfiguration().createDefaultCameraConfiguration();
            cam_config.f_x = j_fx.value().get<double>();
            cam_config.f_y = j_fy.value().get<double>();
            cam_config.c_x = j_cx.value().get<double>();
            cam_config.c_y = j_cy.value().get<double>();
            cam_config.resolution_x = j_resolution_x.value().get<int>();
            cam_config.resolution_y = j_resolution_y.value().get<int>();

            if (j_fps != j_camera.end()) {
                cam_config.fps = j_fps.value().get<double>();
            }
            if (j_focal_x_baseline != j_camera.end()) {
                cam_config.focal_x_baseline = j_focal_x_baseline.value().get<double>();
            }

            cam_config.camera_number = j_number.value().get<uint32_t>();

            if (j_model.value().get<std::string>() == "fisheye") {
                cam_config.distortion_function = LpSlamCameraDistortionFunction_Fisheye;
            } else if (j_model.value().get<std::string>() == "perspective") {
                cam_config.distortion_function = LpSlamCameraDistortionFunction_Pinhole;
            } else if (j_model.value().get<std::string>() == "omni") {
                cam_config.distortion_function = LpSlamCameraDistortionFunction_Omni;
            } else if (j_model.value().get<std::string>() == "no_distortion") {
                cam_config.distortion_function = LpSlamCameraDistortionFunction_NoDistortion;
            } else {
                spdlog::error("Camera model {0} not supported", j_model.value().get<std::string>());
                return false;
            }

            uint32_t i = 0;
            auto j_dist = j_camera.find("distortion");
            if (j_dist == j_camera.end()) {
                // mapping configuration does not need distortion parameters
                spdlog::warn("Camera entry is missing required list 'distortion'");
            }
            else {
                auto j_dist_list = j_dist.value();

                for (uint32_t j = 0; j < LpSlamMaxDistortion; j++) {
                    cam_config.dist[j] = 0.0;
                }
                i = 0;
                for (auto j_one_dist : j_dist_list) {
                    if (i >= LpSlamMaxDistortion) {
                        spdlog::error("Too many distortion parameter in config file");
                        return false;
                    }
                    cam_config.dist[i] = j_one_dist.get<double>();
                    i++;
                }
            }

            // load camera rotation, if configured
            i = 0;
            if (j_rotation != j_camera.end()) {
                auto j_rotation_list = j_rotation.value();

                for ( auto j_one_rotation: j_rotation_list) {
                    if (i >= 9){
                        spdlog::error("Too many rotation parameter in config file");
                        return false;
                    }
                    cam_config.rotation[i] = j_one_rotation.get<double>();
                    i++;
                }
            }

            i = 0;
            if (j_rotation_vec != j_camera.end()) {
                auto j_rotation_vec_list = j_rotation_vec.value();

                cv::Mat rotVec = cv::Mat_<double>(1, 3);
                for ( auto j_one_rotation: j_rotation_vec_list) {
                    if (i >= 3){
                        spdlog::error("Too many rotation vector parameter in config file");
                        return false;
                    }
                    rotVec.at<double>(0, i) = j_one_rotation.get<double>();
                    i++;
                }
                cv::Mat rotMatrix;
                cv::Rodrigues(rotVec /*in*/, rotMatrix/*out*/); 
                // convert to matrix
                std::memcpy(cam_config.rotation, rotMatrix.ptr(0), sizeof(double) * 9);
            }

            // load camera translation
            i = 0;
            if (j_translation != j_camera.end()) {
                auto j_translation_list = j_translation.value();

                cv::Mat tVec = cv::Mat_<double>(1, 3);
                for ( auto j_one_translation: j_translation_list) {
                    if (i >= 3){
                        spdlog::error("Too many translation vector parameter in config file");
                        return false;
                    }
                    cam_config.translation[i] = j_one_translation.get<double>();
                    i++;
                }
            }

            auto j_mask = j_camera.find("mask");
            if (j_mask != j_camera.end()) {
                // note: does not support all mask types yet
                if (j_mask.value().get<std::string>() == "image"){
                    cam_config.mask_type = LpSlamCameraMaskType_Image;
                } else {
                    spdlog::error("Camera mask type {0} not supported",
                        j_mask.value().get<std::string>());
                }
            }

            setCameraConfiguration(cam_config);
        }
    } else {
        spdlog::warn("Config file {0} contains no configured cameras", filename);
    }

    auto j_markers = j_in.find("markers");

    if (j_markers != j_in.end()) {
        auto j_markers_list = j_markers.value();

        // should be a list
        for (auto j_marker : j_markers_list) {
            auto j_type = j_marker.find("type");
            auto j_config = j_marker.find("configuration");

            if (j_type == j_marker.end()) {
                spdlog::error("Marker entry is missing required field 'type'");
                return false;
            }

            std::string configJson;
            if (j_config != j_marker.end()) {
                configJson = j_config.value().dump();
            } else {
                spdlog::warn("Marker entry {0} has no 'configuration' field. Will use default configuration.",
                    filename);
            }

            const auto markerTypeName = j_type.value().get<std::string>();
            {
                spdlog::error("Marker of type {0} not supported", markerTypeName);
            }
        }
    }

    return true;
}

void SlamManager::logToFile(std::string const& filename) {
  const std::string fileLoggerName = "LpSlam_FileLogger";
  // unregister old file logger, if existing
  // because we might write to a different file location now
  if (spdlog::get(fileLoggerName)) {
      spdlog::drop(fileLoggerName);
  }

  auto flogger = spdlog::basic_logger_mt(fileLoggerName, filename);
  spdlog::set_default_logger(flogger);
  spdlog::flush_every(std::chrono::seconds(1));
  spdlog::set_level(spdlog::level::level_enum::info);
  spdlog::info("Enabled output to log file {}", filename);
  setLogLevelInternal();
}

void SlamManager::setLogLevelInternal() {
    if (m_logLevel == LpSlamLogLevel_Debug) {
        spdlog::set_level(spdlog::level::level_enum::debug);
    }
    else if (m_logLevel == LpSlamLogLevel_Info) {
        spdlog::set_level(spdlog::level::level_enum::info);
    }
    else if (m_logLevel == LpSlamLogLevel_Error) {
        spdlog::set_level(spdlog::level::level_enum::warn);
    }
}

void SlamManager::setLogLevel(LpSlamLogLevel level) {
    m_logLevel = level;
    setLogLevelInternal();
}

bool SlamManager::addStereoImageFromBuffer(uint32_t cameraNumber, LpSlamTimestamp timestamp, uint8_t * buffer_left,
    uint8_t * buffer_right,
    LpSlamImageDescription desc) {

    //int32_t cv_imageFormat;

    spdlog::info("Adding stereo images {0} from camera, {1}x{2}, total buffer size {3} at timestamp {4}", cameraNumber,
        desc.width, desc.height,
        desc.imageSize,
        timestamp);

    if (desc.structure == LpSlamImageStructure_Stereo_TwoBuffer) {
        // extract two images
        if (desc.format == LpSlamImageFormat_8UC3) {
            // convert the combined image
            auto cvFrame_left = cv::Mat(desc.height, desc.width,
                    CV_8UC3, buffer_left);
            auto cvConverted_left = cv::Mat(desc.height, desc.width,
                    CV_8UC1);
            cv::cvtColor(cvFrame_left, cvConverted_left, cv::COLOR_RGB2GRAY);

            auto cvFrame_right = cv::Mat(desc.height, desc.width,
                    CV_8UC3, buffer_right);
            auto cvConverted_right = cv::Mat(desc.height, desc.width,
                    CV_8UC1);
            cv::cvtColor(cvFrame_right, cvConverted_right, cv::COLOR_RGB2GRAY);

            // todo: not properly yet
            TrackerCoordinateSystemBase base_left;
            TrackerCoordinateSystemBase base_right;

            auto qEntry =  CameraQueueEntry( int64ToTimeStamp(timestamp),
                cameraNumber,
                base_left, cvConverted_left,
                cameraNumber + 1,
                base_right, cvConverted_right);
            if (desc.hasRosTimestamp > 0) {
                qEntry.ros_timestamp = desc.rosTimestamp;
            }
            m_camQueue.push(qEntry);

            return true;
        } else if (desc.format == LpSlamImageFormat_8UC1) {
            // convert the combined image
            auto cvFrame_left = cv::Mat(desc.height, desc.width,
                    CV_8UC1, buffer_left);
            auto cvFrame_right = cv::Mat(desc.height, desc.width,
                    CV_8UC1, buffer_right);

            // todo: not properly yet
            TrackerCoordinateSystemBase base_left;
            TrackerCoordinateSystemBase base_right;

            auto qEntry = CameraQueueEntry( int64ToTimeStamp(timestamp),
                cameraNumber,
                base_left, cvFrame_left,
                cameraNumber + 1,
                base_right, cvFrame_right);
            if (desc.hasRosTimestamp > 0) {
                qEntry.ros_timestamp = desc.rosTimestamp;
            }
            m_camQueue.push(qEntry);

            return true;
        } else {
            spdlog::error("Image format {0} not supported", desc.format);
            return false;
        }
    } else {
        spdlog::error("Image structure {0} not supported", desc.structure);
    }

    return true;
}

bool SlamManager::addImageFromBuffer(uint32_t cameraNumber, LpSlamTimestamp timestamp, uint8_t * buffer,
  LpSlamImageDescription desc) {

    int32_t cv_imageFormat;

    spdlog::info("Adding images {0} from camera, {1}x{2}, total buffer size {3} at buffer location {4} timestamp {5}", cameraNumber,
        desc.width, desc.height,
        desc.imageSize,
        reinterpret_cast<uint64_t>(buffer),
        timestamp);

    if ( desc.format == LpSlamImageFormat_8UC1_JPEPG ) {
        cv_imageFormat = CV_8UC1;
    } else if ( desc.format == LpSlamImageFormat_8UC4 ) {
        cv_imageFormat = CV_8UC3;
    } else if ( desc.format == LpSlamImageFormat_8UC3 ) {
        cv_imageFormat = CV_8UC3;
    } else if ( desc.format == LpSlamImageFormat_NV12 ) {
        cv_imageFormat = CV_8UC3;
    } else if ( desc.format == LpSlamImageFormat_YUV16 ) {
        cv_imageFormat = CV_8UC3;
    } else {
        spdlog::error("Image format {0} not supported", desc.format);
        return false;
    }

    if (desc.structure == LpSlamImageStructure_OneImage) {
        if (desc.format == LpSlamImageFormat_8UC1_JPEPG) {
            // this is an compressed image from LpGlobalFusion (Webots or recording)
            const std::vector<char> imgDataVect(buffer, buffer + desc.imageSize);
            cv::Mat cvImage = cv::imdecode(imgDataVect, cv::IMREAD_GRAYSCALE);

            // todo: not properly yet
            TrackerCoordinateSystemBase cam_base;

            m_camQueue.push( CameraQueueEntry( int64ToTimeStamp(timestamp),
                cam_base, cvImage));
            return true;
        } else {
            spdlog::error("Image format {0} not supported", desc.format);
            return false;
        }
    }
    else if (desc.structure == LpSlamImageStructure_Stereo_LeftLeft_RightRight) {
        // extract two images
        if (desc.format == LpSlamImageFormat_8UC3) {
            // convert the combined image
            auto cvFrame = cv::Mat(desc.height, desc.width,
                    CV_8UC3, buffer);
            auto cvConverted = cv::Mat(desc.height, desc.width,
                    CV_8UC1);
            cv::cvtColor(cvFrame, cvConverted, cv::COLOR_RGB2GRAY);

            // now split
            auto leftRect = cv::Rect ( 0,              0, desc.width / 2, desc.height);
            auto rightRect = cv::Rect( desc.width / 2, 0, desc.width / 2, desc.height);

            auto cvConvertedRight = cvConverted(rightRect);
            auto cvConvertedLeft = cvConverted(leftRect);

            // todo: not properly yet
            TrackerCoordinateSystemBase base_left;
            TrackerCoordinateSystemBase base_right;

            m_camQueue.push( CameraQueueEntry( int64ToTimeStamp(timestamp),
                cameraNumber,
                base_left, cvConvertedLeft,
                cameraNumber + 1,
                base_right, cvConvertedRight));

            return true;
        }
        else {
            spdlog::error("Image format {0} not supported", desc.format);
            return false;
        }
    }
    else if (desc.structure == LpSlamImageStructure_Stereo_LeftTop_RightBottom) {
        // extract two images
        if ( desc.format == LpSlamImageFormat_NV12) {
            // https://stackoverflow.com/questions/18737842/creating-a-mat-object-from-a-yv12-image-buffer
            // YUV NV12 has 1.5 bytes per pixel, so the output buffer has to be larger

            // upper image is from the right cam on VivePro
            auto cvFrameUpper = cv::Mat(desc.height * 3/2 * 1/2, desc.width,
                    CV_8UC1, buffer);
            auto cvConvertedUpper = cv::Mat( desc.height * 1/2, desc.width,
                    cv_imageFormat);
            cv::cvtColor(cvFrameUpper, cvConvertedUpper, cv::COLOR_YUV2BGR_NV12);

            // upper image is from the left cam on VivePro
            auto cvFrameLower = cv::Mat(desc.height * 3/2 * 1/2, desc.width,
                    CV_8UC1, buffer + (desc.imageSize * 1/2) );
            auto cvConvertedLower = cv::Mat(desc.height * 1/2, desc.width,
                    cv_imageFormat);
            cv::cvtColor(cvFrameLower, cvConvertedLower, cv::COLOR_YUV2BGR_NV12);

            TrackerCoordinateSystemBase base_left;
            TrackerCoordinateSystemBase base_right;

            // the eye line of the HMD headset is in our y-axis
            base_left.position.value[1] = -0.5 * 0.065;
            base_right.position.value[1] = 0.5 * 0.065;

            m_camQueue.push( CameraQueueEntry( int64ToTimeStamp(timestamp),
                cameraNumber,
                base_left, cvConvertedLower,
                cameraNumber + 1,
                base_right, cvConvertedUpper));

            // raw dumping of image for testing
            /*
            std::stringstream sFileOut, sFileOutLower;
            sFileOut << "E:\\tmp\\" << m_numberPic << "_upper.jpg";
            sFileOutLower << "E:\\tmp\\" << m_numberPic << "_lower.jpg";
            m_numberPic++;
            if ((m_numberPic % 100) == 0) {
                cv::imwrite(sFileOut.str(), cvConvertedUpper);
                cv::imwrite(sFileOutLower.str(), cvConvertedLower);
            }*/
            return true;
        } else if (desc.format == LpSlamImageFormat_YUV16) {
            // for Valve Index video encoding, not fully tested yet.
            // YUV Y422 has 2 bytes per pixel, so the output buffer has to be larger
            auto cvFrameUpper = cv::Mat(desc.height * 2 * 1/2, desc.width,
                    CV_8UC1, buffer);
            auto cvConvertedUpper = cv::Mat( desc.height * 1/2, desc.width,
                    cv_imageFormat);
            cv::cvtColor(cvFrameUpper, cvConvertedUpper, cv::COLOR_YUV2RGB_Y422);

            // upper image is from the left cam on VivePro
            auto cvFrameLower = cv::Mat(desc.height * 2 * 1/2, desc.width,
                    CV_8UC1, buffer + (desc.imageSize * 1/2) );
            auto cvConvertedLower = cv::Mat(desc.height * 1/2, desc.width,
                    cv_imageFormat);
            cv::cvtColor(cvFrameLower, cvConvertedLower, cv::COLOR_YUV2RGB_Y422);

            /* raw dumping of image for testing
            std::stringstream sFileOut, sFileOutLower;
            sFileOut << "E:\\tmp\\" << m_numberPic << "_yuv_422_upper.jpg";
            sFileOutLower << "E:\\tmp\\" << m_numberPic << "_yuv_422_lower.jpg";
            m_numberPic++;
            if ((m_numberPic % 100) == 0) {
                cv::imwrite(sFileOut.str(), cvConvertedUpper);
                cv::imwrite(sFileOutLower.str(), cvConvertedLower);
            }
            */
        }
        else {
            spdlog::error("Image format {0} not supported", desc.format);
            return false;
        }
    } else if (desc.structure == LpSlamImageStructure_Stereo_Compressed) {

        if (desc.format == LpSlamImageFormat_8UC1_JPEPG) {

            SPDLOG_DEBUG("Got stereo compressed size left: {0} size right: {1}",
                desc.imageSize, desc.imageSizeSecond);

            // this are 2 stereo image from LpGlobalFusion (Webots or recording)
            const std::vector<char> imgDataVectLeft(buffer, buffer + desc.imageSize);
            cv::Mat cvImageLeft = cv::imdecode(imgDataVectLeft, cv::IMREAD_GRAYSCALE);

            const std::vector<char> imgDataVectRight(buffer + desc.imageSize,
                buffer + desc.imageSize + desc.imageSizeSecond);
            cv::Mat cvImageRight = cv::imdecode(imgDataVectRight, cv::IMREAD_GRAYSCALE);

            // todo: not properly yet
            TrackerCoordinateSystemBase cam_base;

            m_camQueue.push( CameraQueueEntry( int64ToTimeStamp(timestamp),
                0, cam_base, cvImageLeft,
                1, cam_base, cvImageRight));
            return true;
        } else {
            spdlog::error("Image format {0} not supported", desc.format);
            return false;
        }
    } else {
        spdlog::error("Image structure {0} not supported", desc.structure);
        return false;
    }

    return true;
}

#ifdef LPSLAM_BUILD_CHILITAGS
void SlamManager::addMarker(LpSlamMarkerIdentifier id, LpSlamMarkerState state) {
    ChiliTrackerMarkerLoading chiliLoader;

    auto loadedMarker = chiliLoader.createDbEntry(id,
        conversion::positionInterfaceToInternal(state.position),
        conversion::orientationInterfaceToInternal(state.orientation));

    auto dbAccess = m_database.get();
    dbAccess.payload().putEntry(loadedMarker);
}
#else
void SlamManager::addMarker(LpSlamMarkerIdentifier, LpSlamMarkerState) {
}
#endif

#ifdef LPSLAM_BUILD_OPENVSLAM
bool SlamManager::mappingSetMode(bool enableMapping) {
    if (m_vslamTracker == nullptr) {
        return false;
    }

    return m_vslamTracker->mappingSetMode(enableMapping);
}

bool SlamManager::mappingExportCSV(std::string csv_filename) {
    if (m_vslamTracker == nullptr) {
        return false;
    }

    return m_vslamTracker->mappingExportCSV(csv_filename);
}


bool SlamManager::mappingSetFilename(std::string const& filename) {
    if (m_vslamTracker == nullptr) {
        return false;
    }

    return m_vslamTracker->mappingSetFilename(filename);
}

LpSlamStatus SlamManager::getSlamStatus() {
    if (m_vslamTracker != nullptr) {
        auto status = m_vslamTracker->getSlamStatus();
        status.fps = m_currentFps;
        return status;
    } else {
        return LpSlam::support::createLpSlamStatus();
    }
}

std::size_t SlamManager::mappingGetFeatures(LpSlamMapBoundary boundary,
    LpSlamFeatureEntry * entry, std::size_t entry_count, LpSlamMatrix9x9 transform ) {
    if (m_vslamTracker == nullptr) {
        return 0;
    }

    return m_vslamTracker->mappingGetFeatures(boundary, entry, entry_count, transform);
}

std::size_t SlamManager::mappingGetFeaturesCount(LpSlamMapBoundary boundary) {
    if (m_vslamTracker == nullptr) {
        return 0;
    }

    return m_vslamTracker->mappingGetFeaturesCount(boundary);
}

#else

bool SlamManager::mappingSetMode(bool) {
    return false;
}

bool SlamManager::mappingSetFilename(std::string const&) {
    return false;
}

std::size_t SlamManager::mappingGetFeaturesCount(LpSlamMapBoundary) {
    return 0;
}

bool SlamManager::mappingExportCSV(std::string) {
    return false;
}

std::size_t SlamManager::mappingGetFeatures(LpSlamMapBoundary,
    LpSlamFeatureEntry *, std::size_t, LpSlamMatrix9x9) {
    return 0;
}

LpSlamStatus SlamManager::getSlamStatus() {
    return LpSlam::support::createLpSlamStatus();
}

#endif