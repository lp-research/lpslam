#pragma once

#include "ProcessorBase.h"
#include "Utils/ImageProcessing.h"
#include "DataTypes/CameraQueue.h"

#include <future>
#include <optional>

namespace LpSlam {

class BlackoutImageProcessor : public ProcessorBase {
public:

    std::string type() override {
        return "BlackoutImage";
    }

    void processImage(CameraQueueEntry & camEntry) override {

        m_imageCount++;

        /*
        if (!m_firstTimestamp.has_value()) {
            m_firstTimestamp = camEntry.timestamp;
        }

        float timeDistance = float(chr::duration_cast<std::chrono::milliseconds>(
            camEntry.timestamp - m_firstTimestamp.value()).count()) / 1000.0;
        
        if ((timeDistance > m_startDark) && (timeDistance < m_endDark))*/
        if ((m_imageCount > m_imageCountStartDark) && (m_imageCount < m_imageCountEndDark)) {
            // assumes image is grayscale
            camEntry.image.setTo(cv::Scalar(0));//,0,0));
            if (camEntry.image_second.has_value()) {
                camEntry.image_second->setTo(cv::Scalar(0));//,0,0));
            }
        }
    }

    std::optional<TimeStamp> m_firstTimestamp;
    double m_startDark = 79.0;
    double m_endDark = 83.0;

    uint64_t m_imageCount = 0;

    uint64_t m_imageCountStartDark = 150;
    uint64_t m_imageCountEndDark = 190;

};
}