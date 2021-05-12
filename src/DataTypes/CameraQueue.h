#pragma once

#include "DataTypes/Space.h"
#include "Interface/LpSlamTypes.h"

#include <opencv2/core.hpp>
#include <tbb/concurrent_queue.h>

#include <optional>

namespace LpSlam {

    struct CameraQueueEntry {

        CameraQueueEntry() = default;

        CameraQueueEntry(TimeStamp ts, TrackerCoordinateSystemBase b, cv::Mat & im) :
            timestamp(ts), base(b), image(im) {
        }

        CameraQueueEntry(TimeStamp ts,
            LpSlamCameraNumber m_camNumLeft,
            TrackerCoordinateSystemBase b_left, cv::Mat & im_left,
            LpSlamCameraNumber camNumRight,
            TrackerCoordinateSystemBase b_right, cv::Mat & im_right) :
            timestamp(ts),
            cameraNumber(m_camNumLeft),
            base(b_left), image(im_left),
            cameraNumber_second(camNumRight),
            base_second(b_right), image_second(im_right) {
        }

        // Time the image was taken
        TimeStamp timestamp;

        // set if a global state was available when the camer image was stored
        // can be the case when replaying a recording
        std::optional<GlobalState> globalState;

        std::optional<GlobalState> globalState_map;

        LpSlamCameraNumber cameraNumber = 0;

        TrackerCoordinateSystemBase base;

        // cv::Mat is a handle to a reference-counted matrix so its ok to copy it around
        // freely but need to be careful when using in multi-threaded environments
        // In case of stereo camera, this is the left eye
        cv::Mat image;

        LpSlamCameraNumber cameraNumber_second;

        std::optional<TrackerCoordinateSystemBase> base_second;

        // In case of stereo camera, this is the right eye
        std::optional<cv::Mat> image_second;

        std::optional<LpSlamROSTimestamp> ros_timestamp;

        bool valid = true;
    };

    typedef tbb::concurrent_bounded_queue<CameraQueueEntry> CameraQueue;
}