#pragma once

#include <spdlog/spdlog.h>

#include <utility>

namespace LpSlam {

    namespace CustomConfig {
    /*
    inline void addOpenCVcameraOptions(ConfigOptions & opts) {
        opts.required("cameraMatrix", 0.1);
    }*/

        inline std::pair<bool, LpSlamCameraConfiguration> getCameraConfig(LpSlam::CameraRegistryAccess * camReg,
            LpSlamCameraNumber num) {

            LpSlamCameraConfiguration cam = {};

            if (camReg != nullptr) {
                auto access = camReg->get();
                auto configLeft = access.payload().getConfiguration(num);
                if (!configLeft.has_value()) {
                    spdlog::error("Cannot load camera configuration for camera with number {0}", num);
                    return {false, cam};
                }
                cam = configLeft.value();
            } else {
                spdlog::error("Cannot process image without camera image");
                return {false, cam};
            }

            return {true,cam};
        }

        inline std::pair<bool, std::pair<LpSlamCameraConfiguration, LpSlamCameraConfiguration>> getStereoCameraConfig(LpSlam::CameraRegistryAccess * camReg,
            LpSlamCameraNumber leftNum,
            LpSlamCameraNumber rightNum) {

            LpSlamCameraConfiguration camLeft = {};
            LpSlamCameraConfiguration camRight = {};

            if (camReg != nullptr) {
                auto access = camReg->get();
                auto configLeft = access.payload().getConfiguration(leftNum);
                if (!configLeft.has_value()) {
                    spdlog::error("Cannot load camera configuration for left camera with number {0}", leftNum);
                    return {false,{camLeft, camRight}};
                }
                camLeft = configLeft.value();

                auto configRight = access.payload().getConfiguration(rightNum);
                if (!configRight.has_value()) {
                    spdlog::error("Cannot load camera configuration for right camera with number {0}", rightNum);
                    return {false,{camLeft, camRight}};
                }
                camRight = configRight.value();
            } else {
                spdlog::error("Cannot process image without camera image");
                return {false,{camLeft, camRight}};
            }

            return {true,{camLeft, camRight}};
        }

    }
}