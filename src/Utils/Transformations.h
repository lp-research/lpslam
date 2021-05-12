#pragma once

#include "DataTypes/Space.h"
#include "Trackers/TrackerBase.h"
#include "Utils/Math.h"

#include <iostream>

namespace LpSlam {
namespace transformations {

    inline std::pair<Position3, Orientation> transformTrackerToOrigin(TrackerCoordinateSystemBase const& base,
        Position3 measuredPosition, Orientation measuredOrientation) {
        
        // rotate measurement from the camera frame into the global frame
        // by using the camera's orientation
        const Quaternion backRotate = base.orientation.value;

        // correct for camera rotation and translate by camera position
        Vector3 basePosition = (backRotate * measuredPosition.value) + base.position.value;

        // rotate the orientation to the base system
        Quaternion baseOrientation = backRotate * measuredOrientation.value;

        return {basePosition, baseOrientation};
   }
   
    /**
    * transforms a vehicle-relative marker position to the global
    * reference frame
    */
    inline std::pair<Position3, Orientation> transformMarkerToGlobal(Position3 relativeMarkerPositionMeasurement,
        Orientation relativeMarkerOrientationMeasurement,
        Position3 vehiclePosition, Orientation vehicleOrientation) {
        
        // rotate marker measurement to the robot's orientation
        const Vector3 relativeMarkerMeasurementRotated = vehicleOrientation.value * relativeMarkerPositionMeasurement.value;
        const Vector3 markerMeasurementSigmaRotated = Math::posSignVector(vehicleOrientation.value * relativeMarkerPositionMeasurement.sigma);

        // compute the markers position in global frame
        const Vector3 absoluteMarkerPosition = vehiclePosition.value + relativeMarkerMeasurementRotated;

        // rotate marker's orientation to be correct in the global frame
        const Quaternion absoluteMarkerOrientationMeasurement = (vehicleOrientation.value * relativeMarkerOrientationMeasurement.value)
            .normalized();

        return {Position3(absoluteMarkerPosition, markerMeasurementSigmaRotated),
                Orientation(absoluteMarkerOrientationMeasurement, relativeMarkerOrientationMeasurement.sigma)};
    }

    inline std::pair<Position3, Orientation> transformMarkerMeasurementToVehiclePosition(
        // position and orientation measurement relative to the marker
        Position3 relativeMarkerPositionMeasurement,
        Orientation relativeMarkerOrientationMeasurement,
        // marker position in global coordinates
        Position3 markerReferencePosition,
        Orientation markerReferenceOrientation) {

        const Quaternion markerRefOrientConj = markerReferenceOrientation.value.conjugate();

        // rotate marker's orientation to be correct in the global frame
        // the robot has the same orientation as measured by the marker in the global frame
        // conjugate to be valid in the global frame
        const Quaternion absoluteMarkerOrientationMeasurement = (relativeMarkerOrientationMeasurement.value * markerRefOrientConj)
            .conjugate().normalized();

        // rotate marker measurement using the reconstructed robot orientation into the global frame
        const Vector3 relativeMarkerMeasurementRotated = absoluteMarkerOrientationMeasurement * relativeMarkerPositionMeasurement.value;

        // rotated uncertainty on the marker measurement
        // make sure there is no negative sign
        const Vector3 markerMeasurementSigmaRotated = Math::posSignVector(absoluteMarkerOrientationMeasurement * relativeMarkerPositionMeasurement.sigma);

        // compute the vehicle position in global frame
        const Vector3 absoluteMarkerPosition = markerReferencePosition.value - relativeMarkerMeasurementRotated;

        // conjugaret 
        return {
            Position3(absoluteMarkerPosition, markerMeasurementSigmaRotated),
            Orientation(absoluteMarkerOrientationMeasurement, relativeMarkerOrientationMeasurement.sigma)};
    }
}
}
