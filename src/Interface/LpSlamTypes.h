#ifndef LPGFSLAM_TYPES_H
#define LPGFSLAM_TYPES_H

#include <cstdint>

struct LpSlamPosition{
  double x, y, z;
  double x_sigma, y_sigma, z_sigma;
};

struct LpSlamMapPosition {
  // +y right
  double y;
  // +z forward
  double z;
};

struct LpSlamFeaturePosition {
  float x;
  // +y right
  float y;
  // +z forward
  float z;
};

typedef float LpSlamMatrix9x9[9];


struct LpSlamMapBoundary {
  LpSlamMapPosition top_left;
  LpSlamMapPosition bottom_right;
};

struct LpSlamFeatureEntry {
  LpSlamFeaturePosition position;
};

struct LpSlamMapEntry {
  LpSlamMapPosition position;
  // 0.0 = free
  // 1.0 = occupied
  float occupancy;

  // todo: could have
  // - last update
  // - type
};

enum LpSlamLocalization {
  LpSlamLocalization_Off,
  LpSlamLocalization_Initializing,
  LpSlamLocalization_Tracking,
  LpSlamLocalization_Lost
};

enum LpSlamRequestNavDataResult {
  LpSlamRequestNavDataResult_None,
  LpSlamRequestNavDataResult_OdomOnly,
  LpSlamRequestNavDataResult_MapOnly,
  LpSlamRequestNavDataResult_OdomAndMap
};

enum LpSlamNavDataFrame {
  LpSlamNavDataFrame_Camera,
  LpSlamNavDataFrame_Laser,
  LpSlamNavDataFrame_Odometry
};

struct LpMapInfo {
  float x_cell_size;
  float y_cell_size;
  uint32_t x_cell_count;
  uint32_t y_cell_count;
  float x_origin;
  float y_origin;
};

struct LpSlamStatus {
  LpSlamLocalization localization;
  long feature_points;
  long key_frames;

  // in seconds
  double frame_time;

  double fps;
};

struct LpSlamOrientation{
  double w, x, y, z;
  double sigma;
};

typedef uint64_t LpSlamTimestamp;

struct LpSlamROSTimestamp {
    int32_t seconds = 0;
    // internal representation of ROS2
    int64_t nanoseconds = 0;
};

struct LpSlamGlobalState {
  LpSlamPosition position;
  LpSlamOrientation orientation;
  bool valid;
};

struct LpSlamGlobalStateInTime {
  int64_t timestamp;
    LpSlamROSTimestamp ros_timestamp;
    uint8_t has_ros_timestamp;
  LpSlamGlobalState state;
};

using LpSlamRequestNavTransformation = LpSlamGlobalState;

  enum LpSlamLogLevel {
    LpSlamLogLevel_Debug,
    LpSlamLogLevel_Info,
    LpSlamLogLevel_Error
  };

  enum LpSlamImageStructure {
    LpSlamImageStructure_OneImage,
    // image of left camera is on the top of the image and the
    // bottom image is from the right cam
    LpSlamImageStructure_Stereo_LeftTop_RightBottom,
    // image of left cam is to the left, image of right cam
    // is to the right
    LpSlamImageStructure_Stereo_LeftLeft_RightRight,

    // two seperate buffers for the stereo images
    LpSlamImageStructure_Stereo_TwoBuffer,

    // One image in a compressed format
    LpSlamImageStructure_OneImage_Compressed,
    // two images which are individually compressed and
    // one after another in the binary buffer.
    // make sure imageSize and imageSizeSecond are
    // set in the image description 
    LpSlamImageStructure_Stereo_Compressed
  };

  enum LpSlamImageFormat {
    // Grayscale, encoded with JPEG compression
    LpSlamImageFormat_8UC1_JPEPG,
    LpSlamImageFormat_8UC1,
    LpSlamImageFormat_8UC3,
    LpSlamImageFormat_8UC4,
    LpSlamImageFormat_NV12,
    LpSlamImageFormat_YUV16
  };

  enum LpSlamImageConversion {
    LpSlamImageConversion_None,
    LpSlamImageConversion_BGR2RGB
  };

  struct LpSlamImageDescription {
    LpSlamImageStructure structure;
    LpSlamImageFormat format;
    LpSlamImageConversion image_conversion;
    uint32_t height;
    uint32_t width;
    uint32_t imageSize;
    // raw image size of a second image contained in the raw data
    // in case of two images:
    // total buffer size = imageSize + imageSizeSecond
    uint32_t imageSizeSecond;

    uint8_t hasRosTimestamp;
    LpSlamROSTimestamp rosTimestamp;
  };

  enum LpSlamCameraDistortionFunction {
    LpSlamCameraDistortionFunction_Pinhole,
    LpSlamCameraDistortionFunction_Fisheye,
    LpSlamCameraDistortionFunction_Omni,
    LpSlamCameraDistortionFunction_NoDistortion
  };

  enum LpSlamCameraMaskType {
    LpSlamCameraMaskType_None,
    LpSlamCameraMaskType_Radial,
    LpSlamCameraMaskType_Image
  };

  typedef uint32_t LpSlamMarkerIdentifier;

  struct LpSlamMarkerState {
    LpSlamPosition position;
    LpSlamOrientation orientation;
  };

  typedef uint32_t LpSlamCameraNumber;

  const uint32_t LpSlamMaxDistortion = 8;

  struct LpSlamCameraConfiguration {
    LpSlamCameraNumber camera_number;

    LpSlamCameraDistortionFunction distortion_function;

    double f_x;
    double f_y;
    double c_x;
    double c_y;

    double dist[LpSlamMaxDistortion];

    LpSlamCameraMaskType mask_type;
    double mask_parameter;

    int resolution_x;
    int resolution_y;

    double fps;

    // stereo baseline of the camera in focal units
    // focal_x_baseline is -P2[0][3] which is calculated with stereo calibration tool
    // true_baseline = focal_x_baseline / fx
    double focal_x_baseline;

    // rotation of the camera in relation to the stereo rectified plane
    double rotation[9];

    // translation of the camera in relation to the stereo rectified plane
    double translation[3];
  };

typedef void(*OnReconstructionCallback_t)(LpSlamGlobalStateInTime const& reconstructedState, void *);

typedef void(*OnImageCallback_t)(LpSlamTimestamp timestamp, uint32_t cameraNumber, uint8_t * buffer,
      LpSlamImageDescription desc,
      void *);

typedef LpSlamRequestNavDataResult(*RequestNavDataCallback_t)(LpSlamROSTimestamp for_ros_time,
    LpSlamGlobalStateInTime * odometry,
    LpSlamGlobalStateInTime * map,
    void *);

typedef LpSlamRequestNavTransformation (*RequestNavTransformationCallback_t)(LpSlamROSTimestamp ros_time,
        LpSlamNavDataFrame from_frame,
        LpSlamNavDataFrame to_frame,
        void *);

#endif
