#include "CameraCalibrationProcessor.h"

#include "../Utils/StringHelper.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <cmath>

using namespace LpSlam;

void CameraCalibrationProcessor::initializeCorners3d() {
    m_corners_3d.clear();
    m_corners_3d.push_back(std::vector<cv::Point3f>());
    // size of one square in meters, important for correct
    // distance computation, changed from 0.03 to 0.02 to fit on
    // din a4 page
    float squareSize = 0.02;
    for (int i = 0; i < m_boardSize.height; ++i) {
        for (int j = 0; j < m_boardSize.width; ++j) {
            m_corners_3d[0].push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
        }
    }
}

void CameraCalibrationProcessor::processImage(CameraQueueEntry & camEntry) {

    // don't take an image if its too close in time
    if (m_lastTimestamp.has_value()) {
        const auto distTime = camEntry.timestamp - m_lastTimestamp.value();
        if (distTime < std::chrono::seconds(1)) {
            return;
        }
    }
    m_lastTimestamp = camEntry.timestamp;

    spdlog::info("Calibration Status: {0} of needed {1} images sampled", m_corners_2d.size(), m_minImages);
    std::vector<cv::Point2f> pointBuf;

    // please note: this call will block in case the image is very large (2592x1944 RaspiCam image for example)
    auto found = cv::findChessboardCorners(camEntry.image, m_boardSize, pointBuf,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found == 0){
        spdlog::info("No calibration chessboard found in image");
        return;
    }

    // refine
    cv::cornerSubPix(camEntry.image, pointBuf, cv::Size(10,10),
        cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001 ));

    if (m_drawChessboard) {
        cv::drawChessboardCorners(camEntry.image, m_boardSize, pointBuf, found);
    }

    spdlog::info("Calibration chessboard found in image");   

    m_camResolution = cv::Size(camEntry.image.cols, camEntry.image.rows);

    tryCalibrationFit(pointBuf, camEntry.image);
}

void CameraCalibrationProcessor::tryCalibrationFit(std::vector<cv::Point2f> const& points, cv::Mat image) {

    double smallest_summed_distance = std::numeric_limits<double>::max();

    // check if the cheesboard are too close to the corner as this
    // will make the fisheye calibration crash
    // ignore everything within 10% of the border
    const int absMarginX = (float)m_camResolution.width * 0.1;
    const int absMarginY = (float)m_camResolution.height * 0.1;
    for (auto const& p : points) {
        if ( (p.x < absMarginX) || (p.x > ( m_camResolution.width - absMarginX)) ||
             (p.y < absMarginY) || (p.y > ( m_camResolution.height - absMarginY)) ) {
            spdlog::info("Point x = {0} y = {1} too close to image border, skipping the chessboard",
                p.x, p.y);
            return;
        }
    }

    // check if these new points are a novelty
    for (auto const& chessboard: m_corners_2d) {
        // order how the corners are found is determined
        size_t i = 0;
        double summed_distance = 0.0f;
        for (auto const& pt: chessboard) {
            if (i >= points.size()) {
                spdlog::error("Cheesboard sizes do not match ({0} vs {1})",
                    chessboard.size(), points.size());
                continue;
            }

            const auto new_pt = points[i];
            double dist = std::sqrt( 
                std::pow(pt.x - new_pt.x, 2) +
                std::pow(pt.y - new_pt.y, 2));

            SPDLOG_DEBUG("Points distance of point {0} is {1}", i, dist);
            summed_distance += dist;
            i++;
        }

        // normalize by resolution and average
        summed_distance = summed_distance / double(chessboard.size());
        summed_distance = summed_distance / double(m_camResolution.width);

        SPDLOG_DEBUG("Summed distance distance is {0}", summed_distance);
        smallest_summed_distance = std::min(smallest_summed_distance, summed_distance);
    }
    
    SPDLOG_DEBUG("smallest_summed_distance is {0}", smallest_summed_distance);

    if (smallest_summed_distance < m_minDistance) {
        spdlog::info("Not enough difference to previous images, will skip this frame. Summed distance = {0}",
            smallest_summed_distance);
        return;
    }

    spdlog::info("Add new calibration image");

    // remove something old ?
    while (m_corners_2d.size() > m_maxImages) {
        m_corners_2d.erase(m_corners_2d.begin());
    }

    m_corners_2d.push_back(points);

    // store
    std::string outFile = strings::integerToFileNamePadded(m_activeImageNumber) + "_calibration_image.jpg";
    cv::imwrite(outFile, image);
    m_activeImageNumber++;

    if (m_corners_2d.size() < m_minImages) {
        spdlog::info("Not enough images for camera calibration yet. Got {0} images, need {1}",
            m_corners_2d.size(), m_minImages);
    }

    // try fit if sufficient images
    if (m_corners_2d.size() < m_minImages) {
        spdlog::info("Only got {0} images so far. Not sufficient for fit.",
            m_corners_2d.size());
        return;
    }

    const auto termCriteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.00001 );

    initializeCorners3d();
    m_corners_3d.resize(m_corners_2d.size(), m_corners_3d[0]);

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    if (m_lenseType == LenseType::Fisheye) {
        distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

        // this might fail when CALIB_CHECK_COND is used if there are some faulty detections in the
        // image set. use less and better images then
        spdlog::info("Runnning calibration, can take a few seconds ...");
        auto rms = cv::fisheye::calibrate(m_corners_3d, m_corners_2d, m_camResolution, cameraMatrix,
                                     distCoeffs, rvecs, tvecs,
                                    cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW | cv::fisheye::CALIB_CHECK_COND,
                                    termCriteria);
        spdlog::info("Calibration result with {0} images: RMS = {1} camera matrix = \n{2}\n distortion coefficints = \n{3}",
            m_corners_2d.size(), rms, cameraMatrix, distCoeffs);

        cv::Mat undistorted;
        cv::fisheye::undistortImage(image, undistorted, cameraMatrix, distCoeffs,
            // the camera matrix has to be applied again, otherwise the
            // projected coordinates are from 0 t 1
            cameraMatrix);
        cv::imwrite("undistorted_chessboard.jpg", undistorted);
    } else {
        spdlog::error("Lense type not supported for calibration.");
    }
}

