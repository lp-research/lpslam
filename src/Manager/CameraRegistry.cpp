#include "CameraRegistry.h"

#include <spdlog/spdlog.h>

using namespace LpSlam;

void CameraRegistry::setConfiguration(LpSlamCameraConfiguration const& config) {
    m_cams[config.camera_number] = config;
    spdlog::info("Stored configuration for camera {0}. fx: {1} fy: {2} cx: {3} cy: {4} d0: {5} d1: {6} d2: {7} d3: {8} d4: {9} d_func: {10}",
        config.camera_number,
        config.f_x, config.f_y, config.c_x, config.c_y,
        config.dist[0],
        config.dist[1],
        config.dist[2],
        config.dist[3],
        config.dist[4],
        config.distortion_function);
}

std::optional<LpSlamCameraConfiguration> CameraRegistry::getConfiguration(LpSlamCameraNumber num) {
    auto conf_it = m_cams.find(num);
    if (conf_it == m_cams.end()) {
        spdlog::error("Cannot find camera {0} in registry", num);
        return std::nullopt;
    } else {
        return conf_it->second;
    }
}
