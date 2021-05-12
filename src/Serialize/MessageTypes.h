#pragma once

namespace LpSlam {
namespace Serialization {
    enum MessageType {
        CameraImage = 1,
        SensorImu = 2,
        SensorGlobalState = 3,
        Result = 4,
        SensorFeatureList = 5
    };
}
}