#include "RecordEngine.h"

#include "Serialize/SlamSerialize.pb.h"
#include "Serialize/MessageTypes.h"
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <vector>
#include <sstream>

#include <boost/lexical_cast.hpp>

using namespace LpSlam;

namespace {

    template <class TVec3In, class TVec3Out>
    TVec3Out * toProtoVec3( TVec3In const& vec3) {

        auto outPosition = new TVec3Out();
        outPosition->set_x(vec3.value[0]);
        outPosition->set_y(vec3.value[1]);
        outPosition->set_z(vec3.value[2]);

        return outPosition;
    }

    template <class TPosOrient>
    std::pair<LpgfSlamSerialize::Position*, LpgfSlamSerialize::Orientation* >
        toProtoPositionOrientation( TPosOrient const& posOrient) {

        auto outPosition = new LpgfSlamSerialize::Position();
        outPosition->set_x(posOrient.position.value[0]);
        outPosition->set_y(posOrient.position.value[1]);
        outPosition->set_z(posOrient.position.value[2]);
        outPosition->set_x_sigma(posOrient.position.sigma[0]);
        outPosition->set_y_sigma(posOrient.position.sigma[1]);
        outPosition->set_z_sigma(posOrient.position.sigma[2]);

        auto outOrientation = new LpgfSlamSerialize::Orientation();
        outOrientation->set_w(posOrient.orientation.value.w());
        outOrientation->set_x(posOrient.orientation.value.x());
        outOrientation->set_y(posOrient.orientation.value.y());
        outOrientation->set_z(posOrient.orientation.value.z());
        outOrientation->set_sigma(posOrient.orientation.sigma);

        return {outPosition, outOrientation};
    }

}

RecordEngine::~RecordEngine() {
    stop();
}

RecordEngine::RecordEngine() : 
    m_recordThread(
        [](RecordThreadParams params) -> bool {

            const auto lmdStoreState = [] (GlobalState const & state) {
                auto [outPosition, outOrientation] = toProtoPositionOrientation(state);
                auto outState = new LpgfSlamSerialize::GlobalState();
                outState->set_allocated_position(outPosition);
                outState->set_allocated_orientation(outOrientation);
                
                LpgfSlamSerialize::Velocity * velocity = nullptr;

                if (state.velocityValid) {
                    velocity = toProtoVec3<Velocity3, LpgfSlamSerialize::Velocity>(
                        state.velocity);
                } else {
                    velocity = new LpgfSlamSerialize::Velocity();
                }

                return outState;
            };

            // check if there is some work on our work queue
            try {
                RecordQueueEntry recordEntry;
                params.m_q.pop(recordEntry);
                if (recordEntry.valid == false) {
                    return false;
                }

                if (recordEntry.type == EntryType::Camera) {
                    LpgfSlamSerialize::CameraImage outCamEntry;

                    outCamEntry.set_timestamp(recordEntry.camera.timestamp.time_since_epoch().count());

                    std::vector<unsigned char> imgData;
                    cv::imencode(".jpg", recordEntry.camera.image, imgData);

                    outCamEntry.set_imagedata(imgData.data(), imgData.size());
                    outCamEntry.set_cameranumber(recordEntry.camera.cameraNumber);

                    {
                        auto [outBasePosition, outBaseOrientation] = toProtoPositionOrientation(recordEntry.camera.base);

                        auto outBase = new LpgfSlamSerialize::TrackerCoordinateSystem();
                        outBase->set_allocated_position(outBasePosition);
                        outBase->set_allocated_orientation(outBaseOrientation);
                        outCamEntry.set_allocated_imagebase(outBase);
                    }

                    // check if there is a second image
                    if (recordEntry.camera.image_second.has_value()) {
                        imgData.clear();
                        cv::imencode(".jpg", recordEntry.camera.image_second.value(), imgData);

                        outCamEntry.set_imagedata_second(imgData.data(), imgData.size());
                        outCamEntry.set_cameranumber_second(recordEntry.camera.cameraNumber_second);

                        {
                            auto [outBasePosition, outBaseOrientation] = toProtoPositionOrientation(
                                recordEntry.camera.base_second.value());

                            auto outBase = new LpgfSlamSerialize::TrackerCoordinateSystem();
                            outBase->set_allocated_position(outBasePosition);
                            outBase->set_allocated_orientation(outBaseOrientation);
                            outCamEntry.set_allocated_imagebase_second(outBase);
                        }
                    }

                    auto outStateOdom = lmdStoreState(recordEntry.state_odom);
                    outCamEntry.set_allocated_state_odom(outStateOdom);
                    outCamEntry.set_hasglobalstate_odom(recordEntry.state_odom_valid);

                    auto outStateMap = lmdStoreState(recordEntry.state_map);
                    outCamEntry.set_allocated_state_map(outStateMap);
                    outCamEntry.set_hasglobalstate_map(recordEntry.state_map_valid);

                    params.m_stream.toStream( Serialization::CameraImage,
                        outCamEntry, params.m_out.get());

                    if (params.writeRawFile) {
                        std::stringstream sFileOut;

                        std::string rawFileNumber = boost::lexical_cast<std::string>(params.imgCount);
                        const size_t n_zero = 6;
                        // fill with leading zeros
                        rawFileNumber = std::string(n_zero - rawFileNumber.length(), '0') + rawFileNumber;
                        sFileOut << rawFileNumber << "_left.jpg";
                        cv::imwrite(sFileOut.str(), recordEntry.camera.image);

                        if (recordEntry.camera.image_second.has_value()) {
                            std::stringstream sFileOutRight;
                            sFileOutRight << rawFileNumber << "_right.jpg";
                            cv::imwrite(sFileOutRight.str(), recordEntry.camera.image_second.value());
                        }
                    }
                    //cv::imshow("Display Window", recordEntry.camera.image);

                    params.imgCount++;
                } else if (recordEntry.type == EntryType::Sensor) {
                    if (recordEntry.sensor.getSensorType() == SensorQueueEntry::SensorType::Imu) {
                        LpgfSlamSerialize::SensorImu outSensorImu;

                        outSensorImu.set_timestamp(recordEntry.sensor.timestamp.time_since_epoch().count());
                        auto outAcc = toProtoVec3<Acceleration3, LpgfSlamSerialize::Acceleration>(recordEntry.sensor.getAcceleration());
                        auto outGyro = toProtoVec3<AngularVelocity3, LpgfSlamSerialize::AngularVelocity>(recordEntry.sensor.getAngluarVelocity());

                        outSensorImu.set_allocated_acc(outAcc);
                        outSensorImu.set_allocated_gyro(outGyro);

                        params.m_stream.toStream( Serialization::SensorImu,
                            outSensorImu, params.m_out.get());
                    } else if (recordEntry.sensor.getSensorType() == SensorQueueEntry::SensorType::GlobalState) {

                        // right now, just using one ProtoBuf message ...
                        LpgfSlamSerialize::SensorGlobalState outSensorGlobalState;

                        outSensorGlobalState.set_timestamp(recordEntry.sensor.timestamp.time_since_epoch().count());
                        outSensorGlobalState.set_reference(recordEntry.sensor.reference);

                        auto [outPosition, outOrientation] = toProtoPositionOrientation(recordEntry.sensor.getGlobalState());
                        auto outState = new LpgfSlamSerialize::GlobalState();
                        outState->set_allocated_position(outPosition);
                        outState->set_allocated_orientation(outOrientation);
                        outSensorGlobalState.set_allocated_globalstate(outState);

                        params.m_stream.toStream( Serialization::SensorGlobalState,
                            outSensorGlobalState, params.m_out.get());
                    } else if (recordEntry.sensor.getSensorType() == SensorQueueEntry::SensorType::FeatureList) {

                        for (auto const& feature: recordEntry.sensor.getFeatureList()) {
                            LpgfSlamSerialize::SensorFeature outSensorFeature;

                            //std::cout << feature.m_timestamp.time_since_epoch().count() << std::endl;

                            outSensorFeature.set_timestamp(feature.m_timestamp.time_since_epoch().count());
                            outSensorFeature.set_lastobserved(feature.m_lastObserved.time_since_epoch().count());
                            outSensorFeature.set_observationcount(feature.m_observationCount);

                            outSensorFeature.set_allocated_position(toProtoVec3<Position3, LpgfSlamSerialize::Position>
                                (feature.m_position));
                            outSensorFeature.set_allocated_closestkeyframeposition(toProtoVec3<Position3, LpgfSlamSerialize::Position>
                                (feature.m_closestKeyframePosition));

                            outSensorFeature.set_anchorid(feature.m_anchorId);

                            params.m_stream.toStream( Serialization::SensorFeatureList,
                                outSensorFeature, params.m_out.get());
                        }
                    } else {
                        spdlog::error("Sensor type not support for recording");
                    }
                } else if (recordEntry.type == EntryType::Result) {
                        LpgfSlamSerialize::GlobalStateInTime outResult;

                        auto [res_timestamp, res_gs] = recordEntry.result.globalStateInTime;

                        outResult.set_timestamp(res_timestamp.system_time.time_since_epoch().count());

                        auto [outPosition, outOrientation] = toProtoPositionOrientation(res_gs);
                        auto outState = new LpgfSlamSerialize::GlobalState();
                        outState->set_allocated_position(outPosition);
                        outState->set_allocated_orientation(outOrientation);
                        outResult.set_allocated_globalstate(outState);

                        params.m_stream.toStream( Serialization::Result,
                            outResult, params.m_out.get());
                } else {
                    spdlog::error("Recording entry not supported.");
                }
                // pop all from the sensor values

            }
            catch (tbb::user_abort &)
            {
                // all waits on the fusion queue were aborted
                return false;
            }

            // continue thread
            return true;
        }) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
}

void RecordEngine::setOutputFile(std::string const& outputFile) {
    bool startAgain = false;
    if (m_out) {
        stop();
        startAgain = true;
    }

    m_outputFile = outputFile;

    if (startAgain) {
        start();
    }
}

void RecordEngine::setStoreImages(bool b) {
    m_storeImages = b;
}

void RecordEngine::storeSensor( SensorQueueEntry const& sensor) {
    if (!m_out) {
        // not recording
        return;
    }

    RecordQueueEntry entry;
    entry.type = EntryType::Sensor;
    entry.sensor = sensor;

    m_q.push(entry);
}


void RecordEngine::storeCameraImage( CameraQueueEntry const& camera,
    std::optional<GlobalStateInTime> state_odom,
    std::optional<GlobalStateInTime> state_map) {
    if (!m_out || !m_storeImages) {
        // not recording
        return;
    }

    RecordQueueEntry entry;
    entry.type = EntryType::Camera;
    entry.camera = camera;

    if (state_odom.has_value()) {
        entry.state_odom = state_odom.value().second;
        entry.state_odom_valid = true;
    } else {
        entry.state_odom_valid = false;
    }

    if (state_map.has_value()) {
        entry.state_map = state_map.value().second;
        entry.state_map_valid = true;
    } else {
        entry.state_map_valid = false;
    }

    m_q.push(entry);
}

void RecordEngine::storeResult( ResultQueueEntry const& result) {
    if (!m_out) {
        // not recording
        return;
    }

    RecordQueueEntry entry;
    entry.type = EntryType::Result;
    entry.result = result;

    m_q.push(entry);
}

void RecordEngine::setWriteRawFile(bool b) {
    m_writeRawFile = b;
}

void RecordEngine::start() {
    if (m_out) {
        // already recording ...
        return;
    }

    m_out = std::make_shared<std::ofstream>(m_outputFile, std::ofstream::binary);

    m_recordThread.start(RecordThreadParams(m_q, m_stream, m_out, m_imgCount, m_writeRawFile));
    std::ofstream (m_outputFile, std::ios::binary);
}

void RecordEngine::stop() {
    if (m_recordThread.isRunning()) {
        // send the terminate and all previous measurements will be stored
        RecordQueueEntry camEntry;
        camEntry.valid = false;

        m_recordThread.stopAsync();
        m_q.push(camEntry);
        m_recordThread.stop();
        m_q.abort();
        m_q.clear();
    }

    if (m_out) {
        if (m_out->is_open()) {
            *m_out << std::flush;
            m_out->close();
        }
        m_out.reset();
    }
}
