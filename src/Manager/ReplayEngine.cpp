#include "ReplayEngine.h"

#include "Serialize/SlamSerialize.pb.h"
#include "Serialize/ProtoStream.h"
#include "Serialize/MessageTypes.h"

#include <spdlog/spdlog.h>
#include <opencv2/imgcodecs.hpp>

#include <thread>

using namespace LpSlam;

namespace {
    template <class TProtoOrientPos, class TOrientPos>
    void toLpSlamPosOrient( TProtoOrientPos const& protoPosOrient,
            TOrientPos & pos) {
            pos.position = Position3(protoPosOrient.position().x(),
                protoPosOrient.position().y(),
                protoPosOrient.position().z());
            pos.orientation = Orientation(protoPosOrient.orientation().w(),
                protoPosOrient.orientation().x(),
                protoPosOrient.orientation().y(),
                protoPosOrient.orientation().z());
        }

    template <class TProtoOrientPos, class TOrientPos>
    void toLpSlamGlobalState( TProtoOrientPos const& protoGs,
            TOrientPos & gs) {
            toLpSlamPosOrient(protoGs, gs);
            gs.velocity = Velocity3(protoGs.velocity().x(),
                protoGs.velocity().y(),
                protoGs.velocity().z());
            gs.velocityValid = protoGs.velocityvalid();
        }

    template <class TVecIn, class TVecOut>
    void toLpSlamVec3( TVecIn const& vecIn,
            TVecOut & vecOut) {
            vecOut = TVecOut(vecIn.x(),
                vecIn.y(),
                vecIn.z());
        }
}

/*
void ReplayEngine::addModifier(std::unique_ptr< ReplayModifierBase> && modifier) {
    m_modifiers.emplace_back(std::move(modifier));
}
*/

ReplayEngine::ReplayEngine(CameraQueue & q, SensorQueue & sensorQ) :
    m_camQ(q), m_sensorQ(sensorQ) {

}

bool ReplayEngine::loadReplayItems(std::string const& fileName) {
    spdlog::info("Loading replay items from file {}", fileName);

    // try to read the file
    m_istream = std::make_unique<std::ifstream>(fileName, std::ios::binary);
    if (!*m_istream) {
        spdlog::info("Cannot load replay from file {}", fileName);
        return false;
    }

/*    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using Duration = Clock::duration;
*/
    loadNextItems();

    return true;
}

void ReplayEngine::streamMoreReplayItems() {
    if ( m_camQ.size() < (long int)(m_loadImageChunk / 2)) {
        loadNextItems();
    }
}


void ReplayEngine::loadNextItems() {

    if (!m_istream) {
        return;
    }
    if (!m_istream->is_open()) {

    }

    spdlog::debug("Streaming in next set of camera images");

    ProtoStream ps;
    Serialization::MessageType msgType;

    size_t iImageCount = 0;
    FeatureList activeFeatureList;

    while (!m_istream->eof() && (iImageCount < m_loadImageChunk)) {
        ps.fromStreamNextMessageType(m_istream.get(), msgType);

        // something else than the feature list started ? 
        if ((msgType != Serialization::SensorFeatureList) && (activeFeatureList.size() > 0)) {
            // publish !
            TrackerCoordinateSystemBase base;
            m_sensorQ.push(SensorQueueEntry(activeFeatureList[0].m_timestamp, base, activeFeatureList));
            activeFeatureList.clear();
        }

        if (msgType == Serialization::CameraImage) {
            LpgfSlamSerialize::CameraImage inCamera;
            if (!ps.fromStreamNextMessage(m_istream.get(), inCamera)){
                spdlog::critical("Error while reading message type {0} from stream", msgType);
                break;
            }

            TrackerCoordinateSystemBase trackingBase;

            std::unique_ptr<std::string> imgDataPtr(inCamera.release_imagedata());
            const std::vector<char> imgDataVect(imgDataPtr->begin(), imgDataPtr->end());

            auto cvImage = cv::imdecode(imgDataVect, cv::IMREAD_UNCHANGED);

            CameraQueueEntry camEntry(int64ToTimeStamp(inCamera.timestamp()),
                trackingBase, cvImage);

            if (inCamera.has_state_map()) {
                if (inCamera.hasglobalstate_map()) {
                    auto inGs = inCamera.state_map();
                    GlobalState gs;
                    toLpSlamGlobalState(inGs, gs);
                    camEntry.globalState_map = gs;
                }
            }
            if (inCamera.has_state_odom()) {
                if (inCamera.hasglobalstate_odom()) {
                    auto inGs = inCamera.state_odom();
                    GlobalState gs;
                    toLpSlamGlobalState(inGs, gs);
                    camEntry.globalState = gs;
                }
            }

            // extract information for first camera image
            camEntry.cameraNumber = inCamera.cameranumber();
            toLpSlamPosOrient(inCamera.imagebase(), camEntry.base);

            // check if there is a second camera image
            if (inCamera.has_imagebase_second()) {
                std::unique_ptr<std::string> imgDataPtrSecond(inCamera.release_imagedata_second());
                const std::vector<char> imgDataVectSecond(imgDataPtrSecond->begin(), imgDataPtrSecond->end());

                auto cvImage_second = cv::imdecode(imgDataVectSecond, cv::IMREAD_UNCHANGED);
                camEntry.image_second = cvImage_second;
                camEntry.cameraNumber_second = inCamera.cameranumber_second();
                camEntry.base_second = TrackerCoordinateSystemBase();
                auto & baseFromOptional = camEntry.base_second.value();
                toLpSlamPosOrient(inCamera.imagebase_second(), baseFromOptional);
            }

            /*if (iImageCount < 1){
                for (int i=0; i < 2; i++){
                    m_camQ.push(camEntry);
                    iImageCount++;
                    //iImageCount = 100;
                }
            }*/
            /*if ( iImageCount < 5)*/ {
                //for (int i=0; i < 5; i++){
                m_camQ.push(camEntry);
                //}
            }
            iImageCount++;
        } else if (msgType == Serialization::SensorImu) {
            LpgfSlamSerialize::SensorImu inImu;
            if (!ps.fromStreamNextMessage(m_istream.get(), inImu)){
                spdlog::critical("Error while reading message type {0} from stream", msgType);
                break;
            }

            Acceleration3 accOut;
            AngularVelocity3 gyroOut;

            toLpSlamVec3(inImu.acc(), accOut);
            toLpSlamVec3(inImu.gyro(), gyroOut);

            TrackerCoordinateSystemBase base;
            SensorQueueEntry outImu(int64ToTimeStamp(inImu.timestamp()), base,
                accOut, gyroOut);

            m_sensorQ.push(outImu);
        } else if (msgType == Serialization::SensorGlobalState) {
            LpgfSlamSerialize::SensorGlobalState inGlobalState;
            if (!ps.fromStreamNextMessage(m_istream.get(), inGlobalState)){
                spdlog::critical("Error while reading message type {0} from stream", msgType);
                break;
            }

            GlobalState posGlobalState;
            toLpSlamPosOrient(inGlobalState.globalstate(), posGlobalState);

            TrackerCoordinateSystemBase base;
            SensorQueueEntry outGlobalState(int64ToTimeStamp(inGlobalState.timestamp()), base,
                posGlobalState);

            outGlobalState.reference = inGlobalState.reference();

            m_sensorQ.push(outGlobalState);

        } else if (msgType == Serialization::SensorFeatureList) {

            LpgfSlamSerialize::SensorFeature inFeature;
            if (!ps.fromStreamNextMessage(m_istream.get(), inFeature)){
                spdlog::critical("Error while reading message type {0} from stream", msgType);
                break;
            }

            Feature ft;
            ft.m_timestamp = int64ToTimeStamp(inFeature.timestamp());
            ft.m_lastObserved = int64ToTimeStamp(inFeature.lastobserved());
            toLpSlamVec3(inFeature.position(), ft.m_position);
            toLpSlamVec3(inFeature.closestkeyframeposition(), ft.m_closestKeyframePosition);
            ft.m_anchorId = inFeature.anchorid();
            ft.m_observationCount = inFeature.observationcount();

            activeFeatureList.push_back(ft);
        } else if (msgType == Serialization::Result) {
            // ignore result. we dont use it here
            LpgfSlamSerialize::GlobalStateInTime inResult;
            if (!ps.fromStreamNextMessage(m_istream.get(), inResult)){
                spdlog::critical("Error while reading message type {0} from stream", msgType);
                break;
            }
        } else {
            spdlog::critical("Serialized message type {} not supported", msgType);
            //throw std::invalid_argument("Serialized message type not supported");
            // for some recordings, there is some corrupt data at the end, not sure why ^^
            break;
        }
    }

    spdlog::info("Loaded {} replay items", iImageCount);
}
/*
void ReplayEngine::addReplayItem(ReplayItem const& ri) {
	m_items.push(ri);
}
*/
/*
size_t ReplayEngine::replayItemCount() const {
	return m_items.size();
}

TimeDuration ReplayEngine::timeTillNextMeasurement() {
	ReplayItem & nextMeasurement = m_items.front();

    TimeStamp measurementTimestamp = nextMeasurement.measurement.getTimestamp();

    TimeDuration dur(0);
	if (m_lastTimestamp.has_value()) {
		// wait till the time of the measurement
        dur = ( measurementTimestamp - (*m_lastTimestamp));
        // apply replay time modifier
        dur = TimeDuration( TimeDuration::rep( std::round( dur.count() * 1.0 / m_replayTimeFactor)));
    }
	m_lastTimestamp = measurementTimestamp;
    return dur;
}

void ReplayEngine::waitTillNextMeasurement() {
    std::this_thread::sleep_for(timeTillNextMeasurement());
}

std::optional<MeasurementCommonHolder> ReplayEngine::nextMeasurement() {
	if (m_items.empty()) {
		return std::nullopt;
	}

    ReplayItem nextMeasurement = m_items.front();
	// can only pop, once we are done using the reference, 
	// otherwise there will be a memory problem.
	m_items.pop();
	return nextMeasurement.measurement;
}

std::optional<MeasurementCommonHolder> ReplayEngine::peekNextMeasurement() {
    if (m_items.empty()) {
        return std::nullopt;
    }

    ReplayItem nextMeasurement = m_items.front();
    return nextMeasurement.measurement;
}

*/