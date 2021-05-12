#include "ImageSourceBase.h"

using namespace LpSlam;

ImageSourceBase::ImageSourceBase() {
    getConfigOptions().optional(m_configCameraPosition, Position3(0.0, 0.0, 0.0));
    getConfigOptions().optional(m_configCameraOrientation, Orientation(1.0, 0.0, 0.0, 0.0));
}

TrackerCoordinateSystemBase ImageSourceBase::getCoordinateSystemBase() {
    TrackerCoordinateSystemBase base;

    // better to have this in the camera source
    base.position = getConfigOptions().getPosition(m_configCameraPosition);
    base.orientation = getConfigOptions().getOrientation(m_configCameraOrientation);

    return base;
}