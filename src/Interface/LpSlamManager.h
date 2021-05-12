#ifndef LPGFSLAM_MANAGER_H
#define LPGFSLAM_MANAGER_H

/**
* Interface for the LpgfSlam Manage
*/

#include "LpSlamTypes.h"
#include "LpSlamExport.h"

#include <string>

namespace LpSlam{
    class SlamManager;
}

class LPSLAM_EXPORT LpSlamManager {
public:

    LpSlamManager();

    ~LpSlamManager();

    void logToFile(char const* filename);

    void setLogLevel(LpSlamLogLevel);

    void addOnReconstructionCallback(OnReconstructionCallback_t callback, void * userData);

    void addRequestNavDataCallback(RequestNavDataCallback_t callback, void * userData);

    void addRequestNavTransformation(RequestNavTransformationCallback_t callback, void * userData);

    void addOnImageCallback(OnImageCallback_t callback, void * userData);

    void updateGlobalReferenceState(LpSlamGlobalStateInTime globalStateInTime);

    void addImageFromFile(char const* filename);
    void addStereoImageFromFiles(char const* filename_left, char const* filename_right);

    void addMarker(LpSlamMarkerIdentifier id, LpSlamMarkerState state);

    /**
     * If its a stereo image, we assume the second stereo camera is just cameraNumber + 1
     */

    bool addImageFromBuffer(uint32_t cameraNumber, LpSlamTimestamp timestamp, uint8_t * buffer,
      LpSlamImageDescription desc);

    bool addStereoImageFromBuffer(uint32_t cameraNumber, LpSlamTimestamp timestamp, uint8_t * buffer_left,
      uint8_t * buffer_right,
      LpSlamImageDescription desc);

    /*
    buffer out must be as large as buffer in
    */
    static bool compressImage(uint8_t * buffer,
      LpSlamImageDescription desc,
      uint8_t * bufferOut,
      uint32_t * bufferOutSize);

    void setCameraConfiguration(LpSlamCameraConfiguration conf);

    bool readConfigurationFile(char const* filename);

    bool readReplayItems(char const* filename);

    bool addSource(char const* name, char const* config);

    bool addTracker(char const* name, char const* config);

    bool addProcessor(char const* name, char const* config);

    void setShowLiveStream(bool b);

    void setWriteImageFiles(bool b);

    void setRecord(bool b);

    void setRecordImages(bool b);

    void start();

    void stop();

    LpSlamStatus getSlamStatus();

    // mapping API
    void mappingAddLaserScan(LpSlamGlobalStateInTime origin,
      float * ranges, size_t rangeCount,
      float start_range,
      float end_range,
      float start_angle,
      float end_angle,
      float increment,
      // every distance largen than this 
      // is considered to be not a obstacle but just the laser scanner
      // not detecting anything
      float range_threshold);

    unsigned long mappingGetMapRawSize();

    LpMapInfo mappingGetMapRaw(int8_t * map, std::size_t mapSize);

    std::size_t mappingGetFeaturesCount(LpSlamMapBoundary boundary);

    bool mappingSetMode(bool enableMapping);

    bool mappingSetFilename(const char * filename);

    // Exports the currently mapped features to a CSV file
    // returns 0 on success
    bool mappingExportCSV(const char * csv_filename);

  private:
    LpSlam::SlamManager * m_impl;
};

#endif
