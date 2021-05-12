#pragma once

#include "DataTypes/CameraQueue.h"
#include "DataTypes/SensorQueue.h"

#include <string>

namespace LpSlam {

class ProcessorBase {
public:
    virtual ~ProcessorBase() = default;

    virtual void processImage(CameraQueueEntry &) {}

    virtual void processSensorValuesAndResults(std::vector<SensorQueueEntry> const&,
        GlobalStateInTime const&) {}

    virtual std::string type() = 0;

    void setConfig(std::string const&) {
    }
};

}