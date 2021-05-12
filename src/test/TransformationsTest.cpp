
#include <gtest/gtest.h>

#include <spdlog/spdlog.h>

#include "Utils/Transformations.h"
#include "Utils/Math.h"
#include "Utils/StringHelper.h"

using namespace LpSlam;
using namespace LpSlam::transformations;

TEST(transformation, tracker_location)
{
    {
        TrackerCoordinateSystemBase base;

        Position3 measPos = Position3(0.0, 0.0, 3.0);
        Orientation measOrient = Orientation(1.0, 0.0, 0.0, 0.0);

        auto atBase = transformTrackerToOrigin(base, measPos, measOrient);

        // nothing should happen
        ASSERT_NEAR(atBase.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(atBase.first.value[2], 3.0, 0.01);

        ASSERT_NEAR(atBase.second.value.w(), 1.0, 0.01);
    }

    {
        // cam shifted 10 cam to the left
        TrackerCoordinateSystemBase base;
        base.position.value[0] = -10.0;

        Position3 measPos = Position3(0.0, 0.0, 3.0);
        Orientation measOrient = Orientation(1.0, 0.0, 0.0, 0.0);

        auto atBase = transformTrackerToOrigin(base, measPos, measOrient);

        ASSERT_NEAR(atBase.first.value[0], -10.0, 0.01);
        ASSERT_NEAR(atBase.first.value[2], 3.0, 0.01);

        ASSERT_NEAR(atBase.second.value.w(), 1.0, 0.01);
    }

    {
        // cam shifted 10 cam to the right
        TrackerCoordinateSystemBase base;
        base.position.value[0] = 10.0;

        Position3 measPos = Position3(0.0, 0.0, 3.0);
        Orientation measOrient = Orientation(1.0, 0.0, 0.0, 0.0);

        auto atBase = transformTrackerToOrigin(base, measPos, measOrient);

        ASSERT_NEAR(atBase.first.value[0], 10.0, 0.01);
        ASSERT_NEAR(atBase.first.value[2], 3.0, 0.01);

        ASSERT_NEAR(atBase.second.value.w(), 1.0, 0.01);
    }

    {
        TrackerCoordinateSystemBase base;

        // cam rotated by 90 degrees to the left
        Quaternion camRot;
        camRot = Eigen::AngleAxis<double>(Math::toRad(90.0),
            Eigen::Vector3d(1.0, 0.0, 0.0));

        base.orientation = camRot;
        base.position = Position3(0.0, 0.0, 0.0);

        Position3 measPos = Position3(0.0, 0.0, 1.0);
        Position3 rotMeasPos = camRot * measPos.value;
        std::cout << "rot meas " << rotMeasPos.value << std::endl;
        //Position3 measPos = Position3(3.0, 0.0, 0.0);

        // will show rotated in the cam by 90 degrees
        Quaternion measRot;
        measRot = Eigen::AngleAxis<double>(Math::toRad(90.0),
            Eigen::Vector3d(1.0, 0.0, 0.0));
        Orientation measOrient;
        measOrient = measRot;
        
        auto atBase = transformTrackerToOrigin(base, measPos, measOrient);

        std::cout << "atBase pos " << atBase.first.value << std::endl;

        ASSERT_NEAR(atBase.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(atBase.first.value[1], -1.0, 0.01);
        ASSERT_NEAR(atBase.first.value[2], 0.0, 0.01);
    }    

    {
        TrackerCoordinateSystemBase base;

        // cam rotated by 90 degrees to the left and one meter back
        Quaternion camRot;
        camRot = Eigen::AngleAxis<double>(Math::toRad(90.0),
            Eigen::Vector3d(1.0, 0.0, 0.0));

        base.orientation = camRot;
        base.position = Position3(0.0, 0.0, -1.0);

        Position3 measPos = Position3(0.0, 0.0, 3.0);

        // will show rotated in the cam by 90 degrees
        Quaternion measRot;
        measRot = Eigen::AngleAxis<double>(Math::toRad(-90.0),
            Eigen::Vector3d(1.0, 0.0, 0.0));
        Orientation measOrient;
        measOrient = measRot;
        
        auto atBase = transformTrackerToOrigin(base, measPos, measOrient);

        std::cout << "atBase pos " << atBase.first.value << std::endl;

        // should be the same
        ASSERT_NEAR(atBase.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(atBase.first.value[1], -3.0, 0.01);
        ASSERT_NEAR(atBase.first.value[2], -1.0, 0.01);

        ASSERT_NEAR(atBase.second.value.w(), 1.0, 0.01);
    }
}

TEST(transformation, marker_transformation_to_global)
{
    {
        // unrotated vehicle
        const Position3 measPosMarker = Position3(Vector3(0.0, 3.0, 3.0),
            Vector3(0.1, 0.1, 0.1));

        Quaternion rotMarker;
        rotMarker = Eigen::AngleAxis<double>(Math::toRad(45.0),
            Eigen::Vector3d(1.0, 0.0, 0.0));
        Orientation measOrientMarker;
        measOrientMarker = rotMarker;

        const Position3 vehiclePosition = Position3(0.0, 1.0, 1.0);
        const Orientation vehicleOrientation = Orientation(1.0, 0.0, 0.0, 0.0);

        const auto markerInGlobal = transformMarkerToGlobal(measPosMarker, measOrientMarker,
            vehiclePosition, vehicleOrientation);

        // should have placed the marker properly
        ASSERT_NEAR(markerInGlobal.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(markerInGlobal.first.value[1], 4.0, 0.01);
        ASSERT_NEAR(markerInGlobal.first.value[2], 4.0, 0.01);
        // should have keep the errors
        ASSERT_NEAR(markerInGlobal.first.sigma[0], 0.1, 0.01);
        ASSERT_NEAR(markerInGlobal.first.sigma[1], 0.1, 0.01);
        ASSERT_NEAR(markerInGlobal.first.sigma[2], 0.1, 0.01);

        // should be the same after the transformation
        ASSERT_NEAR(markerInGlobal.second.value.angularDistance(measOrientMarker.value), 0.0, 0.01);
    }

    {
        // rotated vehicle
        const Position3 measPosMarker = Position3(0.0, 3.0, 3.0);
        Orientation measOrientMarker;
        measOrientMarker = Orientation(1.0, 0.0, 0.0, 0.0);

        const Position3 vehiclePosition = Position3(0.0, 1.0, 1.0);
        Orientation vehicleOrientation;

        Quaternion vec45rot;
        vec45rot = Eigen::AngleAxis<double>(Math::toRad(45.0),
            Eigen::Vector3d(1.0, 0.0, 0.0));
        // this quates rotates the unit vector in z direction by 45 degrees to
        // the left !!!
        vehicleOrientation = vec45rot;
        //std::cout << (vehicleOrientation * Eigen::Vector3d(0.0, 0.0, 1.0)) << std::endl;

        const auto markerInGlobal = transformMarkerToGlobal(measPosMarker, measOrientMarker,
            vehiclePosition, vehicleOrientation);

        // should have placed the marker properly
        ASSERT_NEAR(markerInGlobal.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(markerInGlobal.first.value[1], 1.0, 0.01);
        // rotated by 45* to the left so all of the camera position measurement goes in the y component
        ASSERT_NEAR(markerInGlobal.first.value[2], 1.0 + measPosMarker.value.norm(), 0.01);

        // should be rotated the same way the vehicle is rotated (vehicle is facing the marker)
        ASSERT_NEAR(markerInGlobal.second.value.angularDistance(vehicleOrientation.value), 0.0, 0.01);
    }
}

TEST(transformation, pose_estimation_looking_at_marker)
{
    {
        // unrotated marker, facing into the room
        // coming from DB
        // Chilitags assumes an unrotated marker to face the camera
        // but more correct is to have the direction of the marker be into
        // the room (+z) so the normals of camera and marker point in the
        // same direction if there is no rotation between the two
        const Position3 refPosition = Position3(0.0, 0.0, 0.0);
        Quaternion refOrientationQuat(0.0, 0.0, 1.0, 0.0);
        Orientation refOrientation(refOrientationQuat);

        // where was the marker measured? this will
        // be in the camera's coordiante system !
        const Position3 measPosition = Position3(0.0, 0.0, 1.0);
        const Orientation measOrientation = Orientation(1.0, 0.0, 0.0, 0.0);
        //Orientation measOrientMarker;
        //measOrientMarker = measOrientation;

/*        const auto markerInGlobal = transformMarkerToGlobal(measPosition, measOrientation,
            refPosition, refOrientation);*/

        // now use this marker placement to reconstruct the robo's position
        const auto camInGlobal = transformMarkerMeasurementToVehiclePosition(measPosition,
            measOrientation,
            refPosition,
            refOrientation);

        std::cout << "Cam position " << camInGlobal.first.value << std::endl;

        ASSERT_NEAR(camInGlobal.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(camInGlobal.first.value[1], 0.0, 0.01);
        ASSERT_NEAR(camInGlobal.first.value[2], 1.0, 0.01);

        Quaternion camRotationExpected;
        camRotationExpected = Eigen::AngleAxis<double>(Math::toRad(180.0),
            Eigen::Vector3d(0.0, 1.0, 0.0));

        // cam orientation should rotated by 180 degrees facing the marker
        ASSERT_NEAR(camInGlobal.second.value.angularDistance(camRotationExpected), 0.0, 0.1);
    }
}

TEST(transformation, marker_transformation_to_global_and_back_consistency)
{
    {
        // unrotated vehicle
        const Position3 measPosMarker = Position3(0.0, 3.0, 3.0);
        Quaternion markerRot;
        markerRot = Eigen::AngleAxis<double>(Math::toRad(90.0),
            Eigen::Vector3d(1.0, 0.0, 0.0));
        Orientation measOrientMarker;
        measOrientMarker = markerRot;

        const Position3 vehiclePosition = Position3(0.0, 1.0, 1.0);
        const Orientation vehicleOrientation = Orientation(1.0, 0.0, 0.0, 0.0);

        const auto markerInGlobal = transformMarkerToGlobal(measPosMarker, measOrientMarker,
            vehiclePosition, vehicleOrientation);

        spdlog::info("Marker Global position {0}", strings::positionToString(markerInGlobal.first));
        spdlog::info("Marker Global orientation {0}", strings::orientationToString(markerInGlobal.second));

        // now use this marker placement to reconstruct the robo's position
        const auto robotInGlobal = transformMarkerMeasurementToVehiclePosition(measPosMarker,
            measOrientMarker,
            markerInGlobal.first,
            markerInGlobal.second);

        std::cout << "Robot position " << robotInGlobal.first.value << std::endl;

        ASSERT_NEAR(robotInGlobal.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(robotInGlobal.first.value[1], 1.0, 0.01);
        ASSERT_NEAR(robotInGlobal.first.value[2], 1.0, 0.01);

        // vehicle position should be the same as before
        ASSERT_NEAR(robotInGlobal.second.value.angularDistance(vehicleOrientation.value), 0.0, 0.01);
    }

    {
        // vehicle is rotated

        // this is the orientation and position as seen from the robot's frame of
        // reference
        const Position3 measPosMarker = Position3(0.0, 3.0, 3.0);
        Orientation measOrientMarker = Orientation(1.0, 0.0, 0.0, 0.0);

        const Position3 vehiclePosition = Position3(0.0, 1.0, 1.0);
        Quaternion rotVehicle;
        rotVehicle = Eigen::AngleAxis<double>(Math::toRad(45.0),
            Eigen::Vector3d(1.0, 0.0, 0.0));
        Orientation vehicleOrientation;
        vehicleOrientation = rotVehicle;

        spdlog::info("Robot Global position input {0}", strings::positionToString(vehiclePosition));
        spdlog::info("Robot Global orientation input {0}", strings::orientationToString(vehicleOrientation));

        const auto markerInGlobal = transformMarkerToGlobal(measPosMarker, measOrientMarker,
            vehiclePosition, vehicleOrientation);

        spdlog::info("Marker Global position {0}", strings::positionToString(markerInGlobal.first));
        spdlog::info("Marker Global orientation {0}", strings::orientationToString(markerInGlobal.second));

        // now use this marker placement to reconstruct the robo's position
        const auto robotInGlobal = transformMarkerMeasurementToVehiclePosition(measPosMarker,
            measOrientMarker,
            markerInGlobal.first,
            markerInGlobal.second);

        spdlog::info("Robot Global position reconstructed {0}", strings::positionToString(robotInGlobal.first));
        spdlog::info("Robot Global orientation reconstructed {0}", strings::orientationToString(robotInGlobal.second));

        ASSERT_NEAR(robotInGlobal.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(robotInGlobal.first.value[1], 1.0, 0.01);
        ASSERT_NEAR(robotInGlobal.first.value[2], 1.0, 0.01);

        // vehicle position should be the same as before
        ASSERT_NEAR(robotInGlobal.second.value.angularDistance(vehicleOrientation.value), 0.0, 0.01);
    }

    {
        spdlog::info("Roateted vehicle and marker");
        // much more complex rotations
        const Position3 measPosMarker = Position3(0.0, 0.0, 0.0);

        Quaternion rotAroundX;
        rotAroundX = Eigen::AngleAxis<double>(Math::toRad(-90.0),
            Eigen::Vector3d(1.0, 0.0, 0.0).normalized());
        Orientation measOrientMarkerX;
        // rotate 45 degrees to the right
        measOrientMarkerX = rotAroundX;

        Quaternion rotAroundY;
        rotAroundY =Eigen::AngleAxis<double>(Math::toRad(-90.0),
            Eigen::Vector3d(0.0, 1.0, 0.0).normalized());

        Orientation measOrientMarker;
        measOrientMarker = rotAroundY * measOrientMarkerX.value;

        const Position3 vehiclePosition = Position3(0.0, 0.0, 0.0);
        Quaternion rotVehic;
        rotVehic = Eigen::AngleAxis<double>(Math::toRad(90.0),
            //Eigen::Vector3d(0.1, 0.8, 0.2).normalized());
            Eigen::Vector3d(1.0, 0.0, 0.0).normalized());
        Orientation vehicleOrientation;
        // rotate 90 degrees to the left
        vehicleOrientation = rotVehic;

        spdlog::info("Robot Global position input {0}", strings::positionToString(vehiclePosition));
        spdlog::info("Robot Global orientation input {0}", strings::orientationToString(vehicleOrientation));

        spdlog::info("Marker local input {0}", strings::orientationToString(measOrientMarker));

        const auto markerInGlobal = transformMarkerToGlobal(measPosMarker, measOrientMarker,
            vehiclePosition, vehicleOrientation);

        spdlog::info("Marker Global position {0}", strings::positionToString(markerInGlobal.first));
        spdlog::info("Marker Global orientation {0}", strings::orientationToString(markerInGlobal.second));

        // now use this marker placement to reconstruct the robo's position
        const auto robotInGlobal = transformMarkerMeasurementToVehiclePosition(measPosMarker,
            measOrientMarker,
            markerInGlobal.first,
            markerInGlobal.second);

        spdlog::info("Robot Global position reconstructed {0}", strings::positionToString(robotInGlobal.first));
        spdlog::info("Robot Global orientation reconstructed {0}", strings::orientationToString(robotInGlobal.second));

        ASSERT_NEAR(robotInGlobal.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(robotInGlobal.first.value[1], 0.0, 0.01);
        ASSERT_NEAR(robotInGlobal.first.value[2], 0.0, 0.01);

        // vehicle position should be the same as before
        ASSERT_NEAR(robotInGlobal.second.value.angularDistance(vehicleOrientation.value), 0.0, 0.01);
    }

    {
        spdlog::info("Arbitrary rotations");
        // much more complex rotations
        const Position3 measPosMarker = Position3(Vector3(0.0, 3.0, 3.0),
            Vector3(0.1, 0.1, 0.1));
        Quaternion quatRotMeas;
        quatRotMeas = Eigen::AngleAxis<double>(Math::toRad(-41.0),
            Eigen::Vector3d(1.0, 0.2, 0.1).normalized());

        Orientation measOrientMarker(quatRotMeas, 0.23f);

        const Position3 vehiclePosition = Position3(0.0, 1.0, 1.0);
        Orientation vehicleOrientation;
        Quaternion quatRot;
        quatRot = Eigen::AngleAxis<double>(Math::toRad(12.0),
            //Eigen::Vector3d(0.1, 0.8, 0.2).normalized());
            Eigen::Vector3d(1.0, 0.6, 0.4).normalized());

        vehicleOrientation = Orientation(quatRot);

        spdlog::info("Robot Global position input {0}", strings::positionToString(vehiclePosition));
        spdlog::info("Robot Global orientation input {0}", strings::orientationToString(vehicleOrientation));

        const auto markerInGlobal = transformMarkerToGlobal(measPosMarker, measOrientMarker,
            vehiclePosition, vehicleOrientation);

        spdlog::info("Marker Global position {0}", strings::positionToString(markerInGlobal.first));
        spdlog::info("Marker Global orientation {0}", strings::orientationToString(markerInGlobal.second));

        // now use this marker placement to reconstruct the robo's position
        const auto robotInGlobal = transformMarkerMeasurementToVehiclePosition(measPosMarker,
            measOrientMarker,
            markerInGlobal.first,
            markerInGlobal.second);

        spdlog::info("Robot Global position reconstructed {0}", strings::positionToString(robotInGlobal.first));
        spdlog::info("Robot Global orientation reconstructed {0}", strings::orientationToString(robotInGlobal.second));

        ASSERT_NEAR(robotInGlobal.first.value[0], 0.0, 0.01);
        ASSERT_NEAR(robotInGlobal.first.value[1], 1.0, 0.01);
        ASSERT_NEAR(robotInGlobal.first.value[2], 1.0, 0.01);
        std::cout << "robotInGlobal.first.sigma = " << robotInGlobal.first.sigma << std::endl;
        // todo: why is the error on this computation so large ?!?
        ASSERT_NEAR(robotInGlobal.first.sigma[0], 0.1, 0.2);
        ASSERT_NEAR(robotInGlobal.first.sigma[1], 0.1, 0.2);
        ASSERT_NEAR(robotInGlobal.first.sigma[2], 0.1, 0.2);

        // vehicle position should be the same as before
        ASSERT_NEAR(robotInGlobal.second.value.angularDistance(vehicleOrientation.value), 0.0, 0.01);
        ASSERT_NEAR(robotInGlobal.second.sigma, 0.23, 0.01);
    }
}