#pragma once

#include "Interface/LpSlamTypes.h"

#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <optional>
#include <chrono>
#include <utility>

namespace LpSlam {

  using TimeStamp = std::chrono::high_resolution_clock::time_point;
  using TimeDuration = std::chrono::high_resolution_clock::duration;

  class CompositeTimestamp {
  public:
      CompositeTimestamp() = default;

      CompositeTimestamp( TimeStamp sysTime) : system_time(sysTime) {
      }

      TimeStamp system_time;
      std::optional<LpSlamROSTimestamp> ros_timestamp;
  };

  inline TimeStamp int64ToTimeStamp(int64_t i) {
    // start from the epoch
    return TimeStamp() + std::chrono::nanoseconds(i);
  }

  template <class TValueType, class TSigmaType>
  class ValueSigmaPair {
    public:
/*      ValueSigmaPair() = default;

      ValueSigmaPair(TValueType v) : value(v) {
      }
*/
      ValueSigmaPair(TValueType v, TSigmaType s) : value(v), sigma(s) {
      }

      TValueType value;
      TSigmaType sigma;
  };

  using Vector3 = Eigen::Matrix<double, 3, 1>;

  /**
   * 3d position which follows this convention:
   * right handed
   * +z is forward
   * +y is right
   * +x is up
   */
  class Position3 : public ValueSigmaPair<Vector3, Vector3> {
    public:
    Position3() :ValueSigmaPair<Vector3, Vector3>(Vector3::Zero(), Vector3::Zero()) {
    }

    Position3(double x, double y, double z) : 
      ValueSigmaPair<Vector3, Vector3>(Vector3(x, y, z), Vector3::Zero()) {
    }

    Position3(Vector3 v) : ValueSigmaPair<Vector3, Vector3>(v, Vector3::Zero()) {
    }

    Position3(Vector3 v, Vector3 s) : ValueSigmaPair<Vector3, Vector3>(v, s) {
    }
  };

  /**
   * 3d acceleration which follows this convention:
   * +z is forward
   * +y is right
   * +x is up
   */
  class Acceleration3 : public ValueSigmaPair<Vector3, Vector3> {
    public:
    Acceleration3() :ValueSigmaPair<Vector3, Vector3>(Vector3::Zero(), Vector3::Zero()) {
    }

    Acceleration3(double x, double y, double z) :
      ValueSigmaPair<Vector3, Vector3>(Vector3(x, y, z), Vector3::Zero()) {
    }

    Acceleration3(Vector3 v) : ValueSigmaPair<Vector3, Vector3>(v, Vector3::Zero()) {
    }

    Acceleration3(Vector3 v, Vector3 s) : ValueSigmaPair<Vector3, Vector3>(v, s) {
    }
  };

  /**
   * 3d velocity which follows this convention:
   * +z is forward
   * +y is right
   * +x is up
   */
  class Velocity3 : public ValueSigmaPair<Vector3, Vector3> {
    public:
    Velocity3() :ValueSigmaPair<Vector3, Vector3>(Vector3::Zero(), Vector3::Zero()) {
    }

    Velocity3(double x, double y, double z) :
      ValueSigmaPair<Vector3, Vector3>(Vector3(x, y, z), Vector3::Zero()) {
    }

    Velocity3(Vector3 v) : ValueSigmaPair<Vector3, Vector3>(v, Vector3::Zero()) {
    }

    Velocity3(Vector3 v, Vector3 s) : ValueSigmaPair<Vector3, Vector3>(v, s) {
    }
  };

  /**
   * 3d angular velocity which follows this convention:
   * +z is forward
   * +y is right
   * +x is up
   */
  class AngularVelocity3 : public ValueSigmaPair<Vector3, Vector3> {
    public:
    AngularVelocity3() :ValueSigmaPair<Vector3, Vector3>(Vector3::Zero(), Vector3::Zero()) {
    }

    AngularVelocity3(double x, double y, double z) :
      ValueSigmaPair<Vector3, Vector3>(Vector3(x, y, z), Vector3::Zero()) {
    }

    AngularVelocity3(Vector3 v) : ValueSigmaPair<Vector3, Vector3>(v, Vector3::Zero()) {
    }

    AngularVelocity3(Vector3 v, Vector3 s) : ValueSigmaPair<Vector3, Vector3>(v, s) {
    }
  };


  using Quaternion = Eigen::Quaternion<double>;

  /**
   * Orientation defined in the Position3 coordinate system defined above.
   * A null orientation means that the vehicle is pointing straight up (+z) axis
   */
  class Orientation : public ValueSigmaPair<Quaternion, double> {
    public:
    Orientation() : ValueSigmaPair<Quaternion, double>(Quaternion(1.0, 0.0, 0.0, 0.0), 0.0) {

    }

    Orientation(double w, double x, double y, double z) : 
      ValueSigmaPair<Quaternion, double>(Quaternion(w, x, y, z), 0.0) {
    }

    Orientation(Quaternion v) : ValueSigmaPair<Quaternion, double>(v, 0.0) {
    }

    Orientation(Quaternion v, double s) : ValueSigmaPair<Quaternion, double>(v, s) {
    }

  };

  /**
   * The global position and orientation of the tracked object in space
   */ 
  class GlobalState {
    public:
      GlobalState() = default;

      GlobalState(Position3 pos, Orientation orient) : position(pos),
        orientation(orient) {
      }

      GlobalState(std::pair<Position3, Orientation> posOrient) : position(posOrient.first),
        orientation(posOrient.second) {
      }

      Position3 position = Position3(Vector3(0.0, 0.0, 0.0));
      Orientation orientation = Orientation(Quaternion(1.0, 0.0, 0.0, 0.0));
      Velocity3 velocity = Velocity3(Vector3(0.0, 0.0, 0.0));
      bool velocityValid = false;
      // are position and orientation valid ?
      bool stateValid = true;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using GlobalStateInTime = std::pair<CompositeTimestamp, GlobalState>;

  struct TrackerCoordinateSystemBase {
      Position3 position = Position3(Vector3(0.0, 0.0, 0.0));
      Orientation orientation = Orientation(Quaternion(1.0, 0.0, 0.0, 0.0));

      // position and orientation are fixed-size eigen data types
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
