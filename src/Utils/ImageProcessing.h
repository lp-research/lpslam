#pragma once

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//*/

// this code is from
// https://github.com/joshdoe/opencv/blob/1d319f683f6b9a8b0c7cbe2abdc9664f0dac919f/modules/imgproc/src/imadjust.cpp

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ccalib/omnidir.hpp>

#include <Utils/TimeMeasurement.h>

#include <spdlog/spdlog.h>
#include <optional>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

namespace LpSlam {

namespace ImageProcessing {

inline void getDistortionCoeffs(LpSlamCameraConfiguration const& camConfig, cv::Mat & distCoeffs) {
    if (camConfig.distortion_function == LpSlamCameraDistortionFunction_Fisheye) {
        distCoeffs = (cv::Mat_<double>(4,1) <<
            camConfig.dist[0],
            camConfig.dist[1],
            camConfig.dist[2],
            camConfig.dist[3]);
    } else if (camConfig.distortion_function == LpSlamCameraDistortionFunction_Omni) {
        // first entry is xi, other four entries is D
        distCoeffs = (cv::Mat_<double>(5, 1) <<
            camConfig.dist[0],
            camConfig.dist[1],
            camConfig.dist[2],
            camConfig.dist[3],
            camConfig.dist[4]);
    } else if (camConfig.distortion_function == LpSlamCameraDistortionFunction_Pinhole) {
        if (camConfig.dist[5] == 0) {
            // cv::stereoRectify can only take 5
            distCoeffs = (cv::Mat_<double>(5, 1) <<
                camConfig.dist[0],
                camConfig.dist[1],
                camConfig.dist[2],
                camConfig.dist[3],
                camConfig.dist[4]);
        } else {
            distCoeffs = (cv::Mat_<double>(8, 1) <<
                camConfig.dist[0],
                camConfig.dist[1],
                camConfig.dist[2],
                camConfig.dist[3],
                camConfig.dist[4],
                camConfig.dist[5],
                camConfig.dist[6],
                camConfig.dist[7]);
        }
    } else if (camConfig.distortion_function == LpSlamCameraDistortionFunction_NoDistortion) {
        distCoeffs = (cv::Mat_<double>(5,1) <<
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    } else {
        spdlog::error("Distortion function {0} not supported", camConfig.distortion_function);
    }
}

inline void getCameraMatrix(LpSlamCameraConfiguration const& camConfig, cv::Mat & cameraMatrix,
        double scaleWidth = 1.0, double scaleHeight = 1.0) {
    cameraMatrix = (cv::Mat_<double>(3,3) <<
        camConfig.f_x * scaleWidth, 0.0f, camConfig.c_x * scaleWidth,
        0.0f, camConfig.f_y * scaleHeight, camConfig.c_y * scaleHeight,
        0.0f, 0.0f, 1.0f);
}

inline void getRotation(LpSlamCameraConfiguration const& camConfig, cv::Mat & rotation) {
    rotation = (cv::Mat_<double>(3,3) <<
        camConfig.rotation[0], camConfig.rotation[1], camConfig.rotation[2],
        camConfig.rotation[3], camConfig.rotation[4], camConfig.rotation[5],
        camConfig.rotation[6], camConfig.rotation[7], camConfig.rotation[8]);
}

inline void getTranslation(LpSlamCameraConfiguration const& camConfig, cv::Mat & translation) {
    translation = (cv::Mat_<double>(3, 1) <<
        camConfig.translation[0], camConfig.translation[1], camConfig.translation[2]);
}

/**
 * This class cashes the undistortion map because creating this map is 95% of the runtime
 * of the cv::undistort call
 */
class Undistort {

public:

    void undistort(LpSlamCameraConfiguration const& leftCam,
        LpSlamCameraConfiguration const& rightCam,
        bool isLeftCam,
        cv::Mat const& image,
        cv::Mat & image_undistorted) {
        if (!m_initialized &&
            leftCam.distortion_function != LpSlamCameraDistortionFunction_NoDistortion) {
            cv::Mat distCoeffs_left;
            cv::Mat distCoeffs_right;
            cv::Mat camMatrix_left;
            cv::Mat camMatrix_right;

            // stereo parameters
            cv::Mat rotation;
            cv::Mat translation;

            getCameraMatrix(leftCam, camMatrix_left);
            getCameraMatrix(rightCam, camMatrix_right);
            getDistortionCoeffs(leftCam, distCoeffs_left);
            getDistortionCoeffs(rightCam, distCoeffs_right);

            getRotation(leftCam, rotation);
            getTranslation(leftCam, translation);

            spdlog::info("Loaded parameters for camera retification: \n is left {0} rotation \n {1} \n translation \n {2}",
                isLeftCam, rotation, translation);

            auto imageSize = cv::Size(leftCam.resolution_x, leftCam.resolution_y);

            // this code is following this convention:
            // https://github.com/stereolabs/zed-open-capture/blob/fix_doc/examples/include/calibration.hpp#L4211

            cv::Mat camMatrix_toUse;
            cv::Mat distCoeffs_toUse;

            if (isLeftCam) {
                camMatrix_toUse = camMatrix_left;
                distCoeffs_toUse = distCoeffs_left;
            }
            else {
                camMatrix_toUse = camMatrix_right;
                distCoeffs_toUse = distCoeffs_right;
            }

            if ((leftCam.distortion_function == LpSlamCameraDistortionFunction_Fisheye) ||
                (leftCam.distortion_function == LpSlamCameraDistortionFunction_Pinhole)) {

                cv::Mat R1, R2, P1, P2, Q;
                cv::stereoRectify(camMatrix_left, distCoeffs_left, camMatrix_right, distCoeffs_right, imageSize, rotation, translation,
                    R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, imageSize);

                cv::Mat R_toUse;
                cv::Mat P_toUse;

                if (isLeftCam) {
                    R_toUse = R1;
                    P_toUse = P1;
                }
                else {
                    R_toUse = R2;
                    P_toUse = P2;
                }

                spdlog::info("Is left camera: {0} computed R : {1} P {2}", isLeftCam, R_toUse, P_toUse);

                if (leftCam.distortion_function == LpSlamCameraDistortionFunction_Fisheye) {
                    cv::fisheye::initUndistortRectifyMap(camMatrix_toUse, distCoeffs_toUse, R_toUse,
                        P_toUse, imageSize, CV_32FC1,
                        m_remap1, m_remap2);
                }
                else if (leftCam.distortion_function == LpSlamCameraDistortionFunction_Pinhole) {
                    cv::initUndistortRectifyMap(camMatrix_toUse, distCoeffs_toUse, R_toUse,
                        P_toUse, imageSize, CV_32FC1,
                        m_remap1, m_remap2);
                }
            }
            else if (leftCam.distortion_function == LpSlamCameraDistortionFunction_Omni) {
                // by convention scale the omni matrix by resolution
                cv::Mat R_toUse;
                // in Omni, each camera has its own rotation matrix
                if (isLeftCam) {
                    getRotation(leftCam, R_toUse);
                    getCameraMatrix(leftCam, camMatrix_toUse, imageSize.width, imageSize.height);
                }
                else {
                    getRotation(rightCam, R_toUse);
                    getCameraMatrix(rightCam, camMatrix_toUse, imageSize.width, imageSize.height);
                }

                // compute new camera matrix to project to preserve more space
                // cv::Matx33f Knew = cv::Matx33f(
                //    imageSize.width / 1.7, 0, imageSize.width / 2,
                //    0, imageSize.height / 1.7, imageSize.height / 2,
                //    0, 0, 1);

                // extract xi and distortion from this one array
                cv::Mat xi = distCoeffs_toUse.rowRange(cv::Range(0, 1));
                cv::Mat D = distCoeffs_toUse.rowRange(cv::Range(1, 5));
                /*
                only supported in contrib-version of OpenCV
                cv::omnidir::initUndistortRectifyMap(camMatrix_toUse,
                    D, xi, R_toUse,
                    Knew, imageSize, CV_32FC1,
                    m_remap1, m_remap2,
                    cv::omnidir::RECTIFY_PERSPECTIVE);
                */
            }

            m_initialized = true;
        }

        if (leftCam.distortion_function == LpSlamCameraDistortionFunction_NoDistortion) {
            image_undistorted = image;
        } else {
            cv::remap(image, image_undistorted, m_remap1, m_remap2, cv::INTER_LINEAR);
        }
    }

private:
    bool m_initialized = false;
    cv::Mat m_remap1;
    cv::Mat m_remap2;
};

inline void stretchlimFromHist( const cv::MatND& hist, double* low_value,
                     double* high_value, double low_fract, double high_fract,
                     unsigned int histSum)
{
    CV_Assert( low_fract >= 0 && low_fract < 1.0 );
    CV_Assert( low_fract < high_fract && high_fract <= 1.0);

    unsigned int sum;
    unsigned int low_count = low_fract * histSum;
    sum = 0;
    for( unsigned int i = 0; i < static_cast<unsigned int>(hist.rows); i++ ) {
        if (sum >= low_count) {
            *low_value = i;
            break;
        }

        sum += ((float*)hist.data)[i];
    }

    unsigned int high_count = (1 - high_fract) * histSum;
    sum = 0;
    for( int i = hist.rows - 1; i >= 0; i-- ) {
        if (sum >= high_count) {
            *high_value = i;
            break;
        }

        sum += ((float*)hist.data)[i];
    }
}

//TODO: surely something like this already exists
inline int bitsFromDepth( int depth )
{
    if (depth == CV_8U)
        return 8;
    else if (depth == CV_16U)
        return 16;
    else
        return 0;
}

//TODO: handle RGB or force user to do a channel at a time?
//template <class InputArray>
inline void stretchlim(cv::InputArray _image, double* low_value,
                double* high_value, double low_fract = 0.01,
                double high_fract = 0.99 )
{
    cv::Mat image = _image.getMat();

    if (low_fract == 0 && high_fract == 1.0) {
        // no need to waste calculating histogram
        *low_value = 0;
        *high_value = 1;
        return;
    }

    int nPixelValues = 1 << bitsFromDepth( image.depth() );
    int channels[] = { 0 };
    cv::MatND hist;
    int histSize[] = { nPixelValues };
    float range[] = { 0, static_cast<float>(nPixelValues)};
    const float* ranges[] = { range };
    cv::calcHist( &image, 1, channels, cv::Mat(), hist, 1, histSize, ranges );
    
    stretchlimFromHist( hist, low_value, high_value, low_fract, high_fract, image.rows * image.cols );

    //TODO: scaling to 0..1 here, but should be in stretchlimFromHist?
    unsigned int maxVal = (1 << bitsFromDepth( _image.depth() )) - 1;
    *low_value /= maxVal;
    *high_value /= maxVal;
}

/*
Adjust the image's constrast by spreading the darkest and brightest color
to a new range
*/
inline void imadjust(cv::InputArray _src, cv::OutputArray _dst, std::optional<double> low_in,
                   std::optional<double> high_in, double low_out, double high_out )
{
    //CV_Assert( (low_in == 0 || high_in != low_in) && high_out != low_out );

    double use_low_in = 0.0;
    double use_high_in = 1.0;
    //spdlog::info("what low {0} high {1}", use_low_in, use_high_in);

    if (!low_in.has_value() || !high_in.has_value()) {
        stretchlim ( _src, &use_low_in, &use_high_in );
    } else {
        use_low_in = low_in.value();
        use_high_in = high_in.value();
    }

    double alpha = (high_out - low_out) / (use_high_in - use_low_in);
    double beta = high_out - use_high_in * alpha;

    cv::Mat src = _src.getMat();
    int depth;
    if (_dst.empty())
        depth = _src.depth();
    else
        depth = _dst.depth();

    //TODO: handle more than just 8U/16U
    //adjust alpha/beta to handle to/from different depths
    int max_in = (1 << bitsFromDepth( _src.depth() )) - 1;
    int max_out = (1 << bitsFromDepth( _dst.depth() )) - 1;
    // y = a*x*(outmax/inmax) + b*outmax
    alpha *= max_out / max_in;
    beta *= max_out;

    src.convertTo( _dst, depth, alpha, beta );
}
}
}
