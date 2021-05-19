#include "Interface/LpSlamManager.h"
#include "Interface/LpSlamConfiguration.h"

#include "InterfaceImpl/LpSlamConversion.h"
#include "Manager/SlamManager.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <spdlog/spdlog.h>

using namespace LpSlam;


LpSlamCameraConfiguration LpSlamConfiguration::createDefaultCameraConfiguration() {
    LpSlamCameraConfiguration camConfig;

    camConfig.c_x = 0.0f;
    camConfig.c_y = 0.0f;
    camConfig.f_x = 0.0f;
    camConfig.f_y = 0.0f;

    camConfig.camera_number = 0;

    camConfig.distortion_function = LpSlamCameraDistortionFunction_NoDistortion;
    for (uint32_t i = 0; i < LpSlamMaxDistortion; i++) {
        camConfig.dist[i] = 0.0;
    }
    camConfig.mask_type = LpSlamCameraMaskType_None;
    camConfig.mask_parameter = 0.0;

    camConfig.resolution_x = 0;
    camConfig.resolution_y = 0;

    camConfig.fps = 25.0;
    camConfig.focal_x_baseline = 0.0;

    for (uint32_t i = 0; i < 9; i++) {
        camConfig.rotation[i] = 0.0;
    }
    camConfig.rotation[0] = 1.0;
    camConfig.rotation[4] = 1.0;
    camConfig.rotation[8] = 1.0;

    camConfig.translation[0] = 0.0;
    camConfig.translation[1] = 0.0;
    camConfig.translation[2] = 0.0;

    return camConfig;
}

LpSlamManager::LpSlamManager() {
  m_impl = new LpSlam::SlamManager();
}

LpSlamManager::~LpSlamManager() {
  delete m_impl;
}

void LpSlamManager::start() {
  m_impl->start();
}

void LpSlamManager::stop() {
 m_impl->stop();
}

bool LpSlamManager::addSource(char const* name, char const* config) {
  return m_impl->addSource(std::string(name), std::string(config));
}

bool LpSlamManager::readReplayItems(char const* filename) {
  return m_impl->loadReplayItems(std::string(filename));
}

void LpSlamManager::logToFile(char const* filename) {
  m_impl->logToFile(filename);
}

void LpSlamManager::setLogLevel(LpSlamLogLevel level) {
  m_impl->setLogLevel(level);
}

void LpSlamManager::addOnReconstructionCallback(OnReconstructionCallback_t callback, void * userData) {
  m_impl->addOnReconstructionCallback(callback, userData);
}

void LpSlamManager::addRequestNavTransformation(RequestNavTransformationCallback_t callback, void * userData) {
  m_impl->addRequestNavTransformationCallback(callback, userData);
}

void LpSlamManager::addRequestNavDataCallback(RequestNavDataCallback_t callback, void * userData) {
    m_impl->addRequestNavDataCallback(callback, userData);
}

void LpSlamManager::addOnImageCallback(OnImageCallback_t callback, void * userData) {
  m_impl->addOnImageCallback(callback, userData);
}

bool LpSlamManager::addTracker(char const* name, char const* config) {
  return m_impl->addTracker(name, config);
}

bool LpSlamManager::addProcessor(char const* name, char const* config) {
  return m_impl->addProcessor(name, config);
}

void LpSlamManager::updateGlobalReferenceState(LpSlamGlobalStateInTime globalStateInTime) {
  //std::cout << "got reference measurement" << std::endl;
  m_impl->updateGlobalReferenceState(conversion::gsInTimeInterfaceToInternal(globalStateInTime));
}

void LpSlamManager::addImageFromFile(char const* filename) {
  auto fsource = m_impl->getFileSource();
  if (fsource == nullptr) {
    spdlog::error("Cannot add image because file source has not been registered.");
  } else {
    fsource->addImage(std::string(filename));
  }
}

void LpSlamManager::addStereoImageFromFiles(char const* filename_left, char const* filename_right) {
    auto fsource = m_impl->getFileSource();
    if (fsource == nullptr) {
        spdlog::error("Cannot add image because file source has not been registered.");
    }
    else {
        fsource->addStereoImage(std::string(filename_left), std::string(filename_right));
    }

}

bool LpSlamManager::compressImage(uint8_t * buffer,
  LpSlamImageDescription desc,
  uint8_t * bufferOut,
  uint32_t * bufferOutSize) {

  // todo: only supports images coming from Webots atm
  cv::Mat camImageBGRA(cv::Size(desc.width, desc.height), CV_8UC4, (void*)buffer);
  cv::Mat camGray;
  cv::Mat camGrayEncoded;
  cv::cvtColor(camImageBGRA, camGray, cv::COLOR_BGRA2GRAY);

  std::vector<unsigned char> imgBuffer;
  cv::imencode(".jpg", camGray, imgBuffer);

  // args. too many copies ...
  std::copy(imgBuffer.begin(), imgBuffer.end(), bufferOut);
  *bufferOutSize = imgBuffer.size();

  return true;
}

void LpSlamManager::setCameraConfiguration(LpSlamCameraConfiguration conf) {
  return m_impl->setCameraConfiguration(conf);
}

bool LpSlamManager::readConfigurationFile(char const* filename) {
  return m_impl->readConfigurationFile(std::string(filename));
}

bool LpSlamManager::addImageFromBuffer(uint32_t cameraNumber, LpSlamTimestamp timestamp, uint8_t * buffer,
  LpSlamImageDescription desc) {
  return m_impl->addImageFromBuffer(cameraNumber, timestamp, buffer, desc);
}

bool LpSlamManager::addStereoImageFromBuffer(uint32_t cameraNumber, LpSlamTimestamp timestamp,
  uint8_t * buffer_left,
  uint8_t * buffer_right,
  LpSlamImageDescription desc) {
  return m_impl->addStereoImageFromBuffer(cameraNumber, timestamp, buffer_left, buffer_right, desc);
}


void LpSlamManager::addMarker(LpSlamMarkerIdentifier id, LpSlamMarkerState state) {
  return m_impl->addMarker(id, state);
}

void LpSlamManager::setShowLiveStream(bool b) {
  return m_impl->setShowLiveStream(b);
}

void LpSlamManager::setWriteImageFiles(bool b) {
  return m_impl->setWriteImageFiles(b);
}

void LpSlamManager::setRecord(bool b) {
  return m_impl->setRecord(b);
}

void LpSlamManager::setRecordImages(bool b) {
  return m_impl->setRecordImages(b);
}

void LpSlamManager::mappingAddLaserScan(LpSlamGlobalStateInTime origin,
      float * ranges, size_t rangeCount,
      float start_range,
      float end_range,
      float start_angle,
      float end_angle,
      float increment,
      float range_threshold) {

  const auto internal_origin = conversion::gsInTimeInterfaceToInternal(origin);

  return m_impl->mappingAddLaserScan(internal_origin,
    ranges, rangeCount,
    start_range, end_range,
    start_angle, end_angle, increment, range_threshold);
}

unsigned long LpSlamManager::mappingGetMapRawSize() {
  return m_impl->mappingGetMapRawSize();
}

LpMapInfo LpSlamManager::mappingGetMapRaw(int8_t * map, std::size_t mapSize) {
  return m_impl->mappingGetMapRaw(map, mapSize);
}

std::size_t LpSlamManager::mappingGetFeatures(LpSlamMapBoundary boundary,
      LpSlamFeatureEntry * entry, std::size_t entry_count, LpSlamMatrix9x9 transform ) {
  return m_impl->mappingGetFeatures(boundary, entry, entry_count, transform);
}

std::size_t LpSlamManager::mappingGetFeaturesCount(LpSlamMapBoundary boundary) {
  return m_impl->mappingGetFeaturesCount(boundary);
}

bool LpSlamManager::mappingSetMode(bool enableMapping) {
  return m_impl->mappingSetMode(enableMapping);
}

bool LpSlamManager::mappingSetFilename(const char * filename) {
  return m_impl->mappingSetFilename(std::string(filename));
}

LpSlamStatus LpSlamManager::getSlamStatus() {
  return m_impl->getSlamStatus();
}

bool LpSlamManager::mappingExportCSV(const char * csv_filename) {
  return m_impl->mappingExportCSV(std::string(csv_filename));  
}
