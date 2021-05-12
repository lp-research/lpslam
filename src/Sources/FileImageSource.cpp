#include "FileImageSource.h"

#include <opencv2/imgcodecs.hpp>

#include <spdlog/spdlog.h>

#include <random>
#include <thread>
#include <iterator>

using namespace LpSlam;

FileImageSource::FileImageSource() : 
    m_worker( [](FileImageSource::WorkerThreadParams params) -> bool {

        ImagePair nextImagePath;
        params.images.pop(nextImagePath);

        // sign to quit
        if (nextImagePath.first.size() == 0) {
            return false;
        }

        cv::Mat image = imread(nextImagePath.first, cv::IMREAD_GRAYSCALE); // Read the file

        std::optional<cv::Mat> image_right;
        if (nextImagePath.second.size() > 0) {
            image_right = imread(nextImagePath.second, cv::IMREAD_GRAYSCALE); // Read the file
            if (image_right->empty()) {
                spdlog::error("Cannot read from right image file {0}", nextImagePath.second);
                return false;
            }
        }

        // dump on image queue
        if (!image.empty()) {
            spdlog::info("Publishing image {0}", nextImagePath.first);

            TrackerCoordinateSystemBase base;
            if (image_right.has_value()) {
                params.camQ.push(CameraQueueEntry(std::chrono::high_resolution_clock::now(),
                    0,
                    base,
                    image,
                    1,
                    base,
                    image_right.value()));
            } else {
                params.camQ.push( CameraQueueEntry(std::chrono::high_resolution_clock::now(),
                    base,
                    image));
            }
        } else{
            spdlog::error("Cannot read from image file {0}",nextImagePath.first);
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (params.loopImages) {
            params.images.push(nextImagePath);
        }

       return true; 
    } ) {
        getConfigOptions().optional(m_configFileNames, std::string());
}

void FileImageSource::addImage(std::string const& imagePath) {
    m_images.push({ imagePath, "" });
}

void FileImageSource::addStereoImage(std::string const& imagePathLeft, std::string const& imagePathRight) {
    m_images.push({ imagePathLeft, imagePathRight });
}

bool FileImageSource::start(CameraQueue & q) {
    auto threadParams = WorkerThreadParams{ q, m_images, m_loopImages };

    m_worker.start(threadParams);
    spdlog::info("Frame aquisition with File Image Source started. Got {0} images in list so far",
        m_images.size());
    return true;
}

void FileImageSource::stop() {
    m_worker.stopAsync();
    // this is the sign to stop
    m_images.clear();
    m_images.push({ "", "" });
    m_worker.stop();

    spdlog::info("Frame aquisition with File Image Source stopped");
}