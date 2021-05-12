#pragma once

#include "DataTypes/Space.h"

#include <vector>
#include <chrono>
#include <utility>

namespace LpSlam {
    class Feature {
        public:
            Feature() = default;

            // for feature points
            Feature(
                TimeStamp timestamp,
                // timestamp when this landmark was last observed
                TimeStamp lastObserved,
                Position3 position,
                Position3 closestKeyframePosition,
                int observationCount )
                : m_timestamp(timestamp), m_lastObserved(lastObserved),
                m_position(position), m_closestKeyframePosition(closestKeyframePosition),
                m_observationCount(observationCount)
                {

            }

            // timestamp when slam algorithm outputted this value
            TimeStamp m_timestamp;
            // timestamp when this landmark was last observed
            TimeStamp m_lastObserved;
            Position3 m_position;
            Position3 m_closestKeyframePosition;
            int m_observationCount = 0;
            
            // if this string is set, this feature represents an anchor point
            std::string m_anchorId;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::vector<Feature> FeatureList;
}
