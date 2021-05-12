#pragma once

#include "DataTypes/CameraQueue.h"
#include "DataTypes/SensorQueue.h"
#include "DataTypes/ResultQueue.h"
#include "Serialize/ProtoStream.h"
#include "Utils/ManagedThread.h"

#include <tbb/concurrent_queue.h>

#include <string>
#include <queue>
#include <optional>
#include <fstream>
#include <memory>
#include <optional>

namespace LpSlam {

class RecordEngine {
public:
    RecordEngine();

    ~RecordEngine();

    void storeCameraImage( CameraQueueEntry const& entry,
        std::optional<GlobalStateInTime> state_odom,
        std::optional<GlobalStateInTime> state_map);

    void storeSensor( SensorQueueEntry const& entry);

    void storeResult( ResultQueueEntry const& entry);

    void setOutputFile(std::string const& outputFile);

    void setWriteRawFile(bool b);

    void setStoreImages(bool b);

    void start();
    void stop();

private:

    enum class EntryType {
        Camera,
        Sensor,
        Result
    };

    struct RecordQueueEntry{ 
        bool valid = true;

        CameraQueueEntry camera;
        SensorQueueEntry sensor;
        GlobalState state_odom;
        bool state_odom_valid;
        GlobalState state_map;
        bool state_map_valid;
        ResultQueueEntry result;
        EntryType type;
    };

    typedef tbb::concurrent_bounded_queue<RecordQueueEntry> RecordQueue;

    RecordQueue m_q;
    // keep the instance here because it needs to allocate quite a large buffer
    ProtoStream m_stream;

    class RecordThreadParams {
    public:
        RecordThreadParams(RecordQueue & q, ProtoStream & stream, std::shared_ptr<std::ofstream> out,
            uint32_t & imgCount_, std::atomic_bool & writeRawFile_) :
            m_q(q), m_stream(stream), m_out(out), imgCount(imgCount_), writeRawFile(writeRawFile_) {

        }

        RecordQueue & m_q;
        ProtoStream & m_stream;
        std::shared_ptr<std::ofstream> m_out;
        uint32_t & imgCount;
        std::atomic_bool & writeRawFile;
    };

    ManagedThread < RecordThreadParams > m_recordThread;
    std::string m_outputFile;
    std::shared_ptr<std::ofstream> m_out;
    uint32_t m_imgCount = 0;
    std::atomic_bool m_writeRawFile = false;
    std::atomic_bool m_storeImages = true;
};

}
