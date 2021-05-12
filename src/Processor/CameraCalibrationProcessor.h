#pragma once

#include "ProcessorBase.h"
#include "Utils/ImageProcessing.h"

#include <optional>

namespace LpSlam {

class CameraCalibrationProcessor : public ProcessorBase {
public:

    std::string type() override {
        return "CameraCalibration";
    }

    void processImage(CameraQueueEntry & camEntry) override;

private:

    void tryCalibrationFit(std::vector<cv::Point2f> const& points, cv::Mat image);

    void initializeCorners3d();

    std::optional<TimeStamp> m_lastTimestamp;

    enum class LenseType {Fisheye, Perspective};
    LenseType m_lenseType = LenseType::Fisheye;

    // too many images take too long to fit ..
    size_t m_maxImages = 80;
    size_t m_minImages = 20;

    // values used for calibration
    std::vector<std::vector<cv::Point2f>> m_corners_2d;
    std::vector<std::vector<cv::Point3f>> m_corners_3d;

    //corners_3d.push_back(std::vector<cv::Point3f>());

    // size of one square in meters, important for correct
    // distance computation, changed from 0.03 to 0.02 to fit on
    // din a4 page
    float m_squareSize = 0.02;
    // order is width then height
    const cv::Size m_boardSize = cv::Size(9, 6);

    cv::Size m_camResolution;

    int m_refineWinSize = 10;
    int m_activeImageNumber = 0;

    bool m_drawChessboard = false;

    // new entries for calibrations
    // must be at least this distance away
    const double m_minDistance = 0.05;
};
}