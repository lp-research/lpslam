
#include <gtest/gtest.h>

#include <spdlog/spdlog.h>

#include "Interface/LpSlamManager.h"
#include "Interface/LpSlamTypes.h"
#include "InterfaceImpl/LpSlamConversion.h"

#include <chrono>

using namespace LpSlam;

TEST(interface, type_conversion)
{
    GlobalState gs;
    gs.position = Position3(Vector3(1.0, 2.0, 3.0), Vector3(10.0, 12.0, 13.0));
    gs.orientation = Orientation(Quaternion(0.5, 0.6, 0.7, 0.8), 23.5f);

    const auto timeNow = std::chrono::high_resolution_clock::now();
    const auto gsWithTime = GlobalStateInTime(timeNow, gs);

    const auto gsWithTimeInterface = conversion::gsInTimeInternalToInterface(gsWithTime);
    const auto gsWithTimeBackConvert = conversion::gsInTimeInterfaceToInternal(gsWithTimeInterface);

    ASSERT_EQ(gsWithTime.first.system_time, gsWithTimeBackConvert.first.system_time);

    ASSERT_EQ(gsWithTime.second.position.value, gsWithTimeBackConvert.second.position.value);
    ASSERT_EQ(gsWithTime.second.position.sigma, gsWithTimeBackConvert.second.position.sigma);
    ASSERT_NEAR(gsWithTime.second.orientation.value.angularDistance(gsWithTimeBackConvert.second.orientation.value),
        0.0, 0.01);
    ASSERT_EQ(gsWithTime.second.orientation.sigma, gsWithTimeBackConvert.second.orientation.sigma);
}

TEST(interface, add_marker)
{
    LpSlamManager manager;
    manager.addMarker(23,
        LpSlamMarkerState{
            LpSlamPosition{1.0, 2.0, 3.0, 0.1, 0.1, 0.1},
            LpSlamOrientation{0.0, 0.5, 0.5, 0.0, 0.1}
        });

    // todo: can't test it right now ...
}