#pragma once

#include "DataTypes/Space.h"
#include "Trackers/TrackerBase.h"
#include "Utils/ManagedThread.h"
#include "DataTypes/CameraQueue.h"
#include "Sources/ImageSourceBase.h"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <tbb/concurrent_queue.h>
#include <string>

namespace LpSlam {

class FileImageSource : public ImageSourceBase{
public:

    FileImageSource();

    bool start(CameraQueue &) override;

    void stop() override;

    void addImage(std::string const& imagePath);
    void addStereoImage(std::string const& imagePathLeft, std::string const& imagePathRight);

private:
    typedef std::pair<std::string, std::string> ImagePair;

    struct WorkerThreadParams {
        CameraQueue & camQ;
        tbb::concurrent_bounded_queue<ImagePair> & images;
        bool loopImages;
    };

    ManagedThread<WorkerThreadParams> m_worker;

    tbb::concurrent_bounded_queue<ImagePair> m_images;
    bool m_loopImages = true;

    const std::string m_configFileNames = "file_names";
};

}