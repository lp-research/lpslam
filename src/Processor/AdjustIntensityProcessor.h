#pragma once

#include "ProcessorBase.h"
#include "Utils/ImageProcessing.h"

#include <future>

namespace LpSlam {

class AdjustIntensityProcessor : public ProcessorBase {
public:

    std::string type() override {
        return "AdjustIntensity";
    }

    void processImage(CameraQueueEntry & camEntry) override {
        // do each image multi-threaded
        cv::Mat image = camEntry.image;
        auto img_thread = std::async(std::launch::async, [image](){
            cv::Mat image_out;
            // over/undersaturate a bit too.
            ImageProcessing::imadjust(image, image_out, std::nullopt, std::nullopt,
                -0.3, 1.4);
            return image_out;
        });

        if (camEntry.image_second.has_value()) {
            cv::Mat image_second = camEntry.image_second.value();
            auto img_thread_second = std::async(std::launch::async, [image_second](){
                cv::Mat image_out;
                // over/undersaturate a bit too.
                ImageProcessing::imadjust(image_second, image_out, std::nullopt, std::nullopt,
                -0.3, 1.4);
                return image_out;
            });
            camEntry.image_second = img_thread_second.get();
        }
        camEntry.image = img_thread.get();
    }
};
}