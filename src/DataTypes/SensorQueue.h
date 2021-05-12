#pragma once

#include "DataTypes/Space.h"
#include "DataTypes/Feature.h"
#include "Interface/LpSlamTypes.h"

#include <opencv2/core.hpp>
#include <tbb/concurrent_queue.h>

#include <optional>

namespace LpSlam {

    struct SensorQueueEntry {

        enum class SensorType {
            Imu,
            GlobalState,
            FeatureList
        };

        SensorQueueEntry() = default;

        SensorQueueEntry(TimeStamp ts, TrackerCoordinateSystemBase b, Acceleration3 acc, AngularVelocity3 gyro) :
            timestamp(ts), base(b) {
            m_sensorType = SensorType::Imu;
            m_acc = acc;
            m_gyro = gyro;
        }

        SensorQueueEntry(TimeStamp ts, TrackerCoordinateSystemBase b, GlobalState state) :
            timestamp(ts), base(b) {
            m_sensorType = SensorType::GlobalState;
            m_globalState = state;
        }

        SensorQueueEntry(TimeStamp ts, TrackerCoordinateSystemBase b, FeatureList const& featureList) :
            timestamp(ts), base(b) {
            m_sensorType = SensorType::FeatureList;
            m_featureList = featureList;
        }

        Acceleration3 getAcceleration() const {
            return m_acc;
        }

        AngularVelocity3 getAngluarVelocity() const {
            return m_gyro;
        }

        GlobalState getGlobalState() const {
            return m_globalState;
        }

        SensorType getSensorType() const {
            return m_sensorType;
        }

        FeatureList const& getFeatureList() const {
            return m_featureList;
        }

        // Time the image was taken
        TimeStamp timestamp;
        TrackerCoordinateSystemBase base;

        bool valid = true;
        // the sensor entry is from a reference tracking system
        // especially useful for global positions
        bool reference = false;

        private:
        SensorType m_sensorType;

        // for IMU
        Acceleration3 m_acc;
        AngularVelocity3 m_gyro;

        // Absolute Position and Orientation
        GlobalState m_globalState;

        FeatureList m_featureList;
    };

    typedef tbb::concurrent_bounded_queue<SensorQueueEntry> SensorQueue;
}