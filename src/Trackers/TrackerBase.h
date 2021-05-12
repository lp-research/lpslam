#pragma once

#include "DataTypes/Space.h"
#include "Utils/ConfigOptions.h"
#include "Manager/CameraRegistry.h"
#include "Interface/LpSlamTypes.h"

#include <opencv2/core.hpp>
#include "DataTypes/CameraQueue.h"
#include "DataTypes/SensorQueue.h"

#include <spdlog/spdlog.h>
#include <string>
#include <vector>
#include <optional>

#include "DataTypes/Marker.h"

namespace LpSlam {

enum class ResultType {
    // the result is the position and orientation of the
    // vehicle in relation to the camera
    TrackedMarker,
    // The result is the absoluto position and orientation of the
    // vehicle in the world
    TrackedVehicle
};

/**
 * Represents a tracking result. The positon and orientation are with the vehicle's
 * fusion position as origin.
 */
struct TrackerResult {

    ResultType type = ResultType::TrackedMarker;

    // unique id associated with a tracker object. can be the result of std::hash
    MarkerId id = 0;

    // this is the relative position of the marker in relation to the origin of
    // the vehicle. This implicitly contains the orientation of the marker in
    // relation to the vehicle
    Position3 position = Position3(Vector3(0.0, 0.0, 0.0));

    Orientation orientation = Orientation(Quaternion(1.0, 0.0, 0.0, 0.0));
    //OrientationSigma orientation_sigma = 0.0f;

    // Time at which the measurement of this tracking result was taken
    // This is the time of the sensor read-out
    CompositeTimestamp timestamp;

    // position and orientation are fixed-size eigen data types
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class TrackerBase {
public:

    virtual ~TrackerBase() = default;

    typedef std::vector<TrackerResult> ProcessImageResult;

    /**
     * Returns the relative position and orientation of the objects in relation to
     * the camera.
     */
    virtual ProcessImageResult processImage(CameraQueueEntry & cam,
        std::optional<GlobalStateInTime> navResultOdom = std::nullopt,
        std::optional<GlobalStateInTime> navResultMap = std::nullopt,
        std::vector<SensorQueueEntry> const& sensorValues = {}) = 0;

    virtual void addRequestNavTransformationCallback([[maybe_unused]]RequestNavTransformationCallback_t callback,
        [[maybe_unused]]void * userData) {
    }

    virtual std::optional<unsigned long> mappingGetMapRawSize() {
        return std::nullopt;
    }

    virtual std::optional<LpMapInfo> mappingGetMapRaw( [[maybe_unused]]int8_t * map,
        [[maybe_unused]]std::size_t mapSize) {
        return std::nullopt;
    }

    virtual void addLaserScan([[maybe_unused]]GlobalStateInTime origin,
        [[maybe_unused]]float * ranges,
        [[maybe_unused]]size_t rangeCount,
        [[maybe_unused]]float start_range, [[maybe_unused]]float end_range,
        [[maybe_unused]]float start_angle, [[maybe_unused]]float end_angle,
        [[maybe_unused]]float increment,
        [[maybe_unused]]float range_threshold) {

    }

    bool setConfig(std::string const& jsonConfig){
        auto newConfig = m_config;

        try {
            newConfig.parse(jsonConfig);
        } catch (std::exception & ex) {
            spdlog::error("Cannot parse config due to error: {0}",
                ex.what());
            return false;
        }

        // if parsing did not throw an exception
        // we can copy the new config over
        m_config = newConfig;
        OnConfigurationUpdate();
        return true;
    }

    virtual bool start(SensorQueue &) {
        return true;
    }

    virtual bool stop() {
        return true;
    }

    /** returns the position and orientation of the coordinate system of
     * this tracker in relation to the origin of the LPGF system. This can
     * be the camera position and orientation in the vehicle coordinate system
     * in a cam-based system
     */
    //virtual TrackerCoordinateSystemBase getCoordinateSystemBase() = 0;

    virtual void OnConfigurationUpdate() {};

    virtual std::string type() = 0;

    void setCameraRegistry(LpSlam::CameraRegistryAccess * camReg) {
        m_camReg = camReg;
    }

    LpSlam::CameraRegistryAccess * getCameraRegistry() {
        return m_camReg;
    }

protected:
    ConfigOptions & getConfigOptions() {
        return m_config;
    }

private:
    ConfigOptions m_config;
    // can be null if never set
    LpSlam::CameraRegistryAccess * m_camReg = nullptr;
};

}