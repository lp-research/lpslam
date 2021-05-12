#ifndef LPSLAM_CONVERSION
#define LPSLAM_CONVERSION

#include "Interface/LpSlamTypes.h"
#include "DataTypes/Space.h"

namespace LpSlam {
namespace conversion{

    // conversion from inferce types to internal types
  inline LpSlam::Position3 positionInterfaceToInternal(LpSlamPosition const &p) {
    return LpSlam::Position3(Vector3(p.x, p.y, p.z), Vector3(p.x_sigma, p.y_sigma, p.z_sigma));
  }

  inline LpSlam::Position3 mapPositionInterfaceToInternal(LpSlamMapPosition const &p) {
    return LpSlam::Position3(Vector3(0.0, p.y, p.z), Vector3(0.0, 0.0, 0.0));
  }

  inline LpSlam::Orientation orientationInterfaceToInternal(LpSlamOrientation const &p) {
    return LpSlam::Orientation(Quaternion(p.w, p.x, p.y, p.z), p.sigma);
  }

  inline LpSlam::GlobalState gsInterfaceToInternal(LpSlamGlobalState const & gs) {
    return LpSlam::GlobalState( positionInterfaceToInternal(gs.position),
        orientationInterfaceToInternal(gs.orientation));
  }

  inline LpSlam::GlobalStateInTime gsInTimeInterfaceToInternal(LpSlamGlobalStateInTime const & gs) {

      const auto durFromTs = std::chrono::high_resolution_clock::duration(gs.timestamp);
      const auto timepoint = std::chrono::high_resolution_clock::time_point(durFromTs);

     auto gs_intime = LpSlam::GlobalStateInTime {
        timepoint,
        gsInterfaceToInternal(gs.state)};

      if (gs.has_ros_timestamp) {
        gs_intime.first.ros_timestamp = gs.ros_timestamp;
      }

      return gs_intime;
  }

  // conversions from internal types to interfaces types
  inline LpSlamPosition positionInternalToInterface(LpSlam::Position3 const &p) {
    return LpSlamPosition{p.value[0], p.value[1], p.value[2], p.sigma[0], p.sigma[1], p.sigma[2]};
  }

  inline LpSlamMapPosition mapPositionInternalToInterface(LpSlam::Position3 const &p) {
    return LpSlamMapPosition{p.value[1], p.value[2]};
  }

  inline LpSlamOrientation orientationInternalToInterface(LpSlam::Orientation const &p) {
    return LpSlamOrientation{p.value.w(), p.value.x(), p.value.y(), p.value.z(), p.sigma};
  }

  inline LpSlamGlobalState gsInternalToInterface(LpSlam::GlobalState const & gs) {
    return LpSlamGlobalState{ positionInternalToInterface(gs.position),
        orientationInternalToInterface(gs.orientation),
        gs.stateValid};
  }

  inline LpSlamGlobalStateInTime gsInTimeInternalToInterface(LpSlam::GlobalStateInTime const & gs) {
    
    LpSlamGlobalStateInTime lpgs;
    lpgs.timestamp = gs.first.system_time.time_since_epoch().count();
    lpgs.state = gsInternalToInterface(gs.second);

    lpgs.has_ros_timestamp = gs.first.ros_timestamp.has_value() ? 1 : 0;
    if (gs.first.ros_timestamp.has_value()) {
      lpgs.ros_timestamp = gs.first.ros_timestamp.value();
    }

    return lpgs;
  }

}
}

#endif
