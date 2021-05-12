#pragma once 

#include "DataTypes/CameraQueue.h"
#include "DataTypes/SensorQueue.h"
#include "Utils/ConfigOptions.h"
#include "Manager/CameraRegistry.h"

namespace LpSlam {

class ImageSourceBase{
public:

    ImageSourceBase();

    virtual ~ImageSourceBase() = default;

    virtual bool start(CameraQueue &) = 0;

    virtual bool startSensor(SensorQueue &) { return true; };

    virtual void stop() = 0;

    virtual TrackerCoordinateSystemBase getCoordinateSystemBase();

    void setConfig(std::string const& conf) {
        auto newConfig = m_config;

        newConfig.parse(conf);

        // if parsing did not throw an exception
        // we can copy the new config over
        m_config = newConfig;
        //onConfigurationChanged();
    }

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


protected:
    /** position of the camera on the vehicle */
    const std::string m_configCameraPosition = "camera_position";
    /** oreintation of the camera on the vehicle */
    const std::string m_configCameraOrientation = "camera_orientation";

private:
    ConfigOptions m_config;
    // can be null if never set
    LpSlam::CameraRegistryAccess * m_camReg = nullptr;
};
}