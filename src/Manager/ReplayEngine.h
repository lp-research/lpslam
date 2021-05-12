#pragma once

#include "DataTypes/CameraQueue.h"
#include "DataTypes/SensorQueue.h"

#include <string>
#include <queue>
#include <optional>
#include <atomic>
#include <fstream>

namespace LpSlam {

class ReplayEngine {
public:
    ReplayEngine(CameraQueue & camQ, SensorQueue & sensorQ);

    bool loadReplayItems(std::string const& fileName);

    void streamMoreReplayItems();
/*
    void addReplayItem(ReplayItem const& ri);

    size_t replayItemCount() const;

    void waitTillNextMeasurement();

    std::optional<MeasurementCommonHolder> nextMeasurement();

    std::optional<MeasurementCommonHolder> peekNextMeasurement();

    void setReplayTimeFactor(double tf) {
        m_replayTimeFactor = tf;
    }

    TimeDuration timeTillNextMeasurement();

    void addModifier(std::unique_ptr< ReplayModifierBase> && modifier);
*/
    void setLoadImageChunk(size_t imageChunk) {
        m_loadImageChunk = imageChunk;
    }

private:

    void loadNextItems();

    CameraQueue & m_camQ;
    SensorQueue & m_sensorQ;

    std::unique_ptr<std::ifstream> m_istream;

    size_t m_loadImageChunk = 500;
    //std::optional<TimeStamp> m_lastTimestamp;
    //std::queue<ReplayItem> m_items;
    //std::atomic<double> m_replayTimeFactor = 1.0;

    //std::vector <std::unique_ptr< ReplayModifierBase>> m_modifiers;
};

}