#include "LpSlamManager.h"
#include "Utils/FileSystem.h"

#include <boost/program_options.hpp>

#include <iostream>
#include <thread>

namespace po = boost::program_options;

void OnReconstructionCallback(LpSlamGlobalStateInTime const&, void *) {
/*    std::cout << "Got Reco Callback at " << reconstructedState.state.position.x <<
        " " << reconstructedState.state.position.y <<
        " " << reconstructedState.state.position.z << std::endl;*/
}

int main(int argc, char *argv[])
{
    po::variables_map vm;

    try
    {
        po::options_description desc("Allowed options");
        desc.add_options()("help", "Show usage")
            ("images", po::value<std::string>(), "Wilcard to images used for calibration")
            ("replay", po::value<std::string>(), "replay from file")
            ("logfile", po::value<std::string>(), "log to file")
            ("record", "record data")
            ("record-no-video", "record no images, only sensor data and tracking results")
            ("verbose", "some debug output")
            ("verbose-debug", "more debug output")
            ("show-live", "show live images of camera input")
            ("store-images", "store images of camera input")
            ("config", po::value<std::string>(), "Load config file");

        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << desc << "\n";
            return 0;
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
    
    LpSlamManager mgr;

    if (vm.count("verbose-debug")) {
        mgr.setLogLevel(LpSlamLogLevel_Debug);
    } else if (vm.count("verbose")) {
        mgr.setLogLevel(LpSlamLogLevel_Info);
    }

    if (vm.count("show-live")) {
        mgr.setShowLiveStream(true);
    }

    if (vm.count("store-images")) {
        mgr.setWriteImageFiles(true);
    }

    if (vm.count("logfile")) {
        mgr.logToFile(vm["logfile"].as<std::string>().c_str());
    }

    if (vm.count("config")) {
        mgr.readConfigurationFile(vm["config"].as<std::string>().c_str());
    }

    if (vm.count("replay")) {
        mgr.readReplayItems(vm["replay"].as<std::string>().c_str());
    }

    if (vm.count("record") > 0) {
        mgr.setRecord(true);
    }

    if (vm.count("record-no-video") > 0) {
        mgr.setRecordImages(false);
    }

    std::string imageGlobPath = "";
/*    std::vector<std::filesystem::path> foundFiles;
    if (vm.count("images") > 0)
    {   
        foundFiles = LpSlam::FileSystem::imagesInFolder(vm["images"].as<std::string>());
    }

    if (foundFiles.size() > 0) {
        // need to add a file image source
        mgr.addSource("FileSource", "");
    }

    std::for_each(foundFiles.begin(), foundFiles.end(), [&] (auto const& f) {
        std::cout << f.u8string() << std::endl;
        mgr.addImageFromFile(f.u8string().c_str());
    });
  */  

    mgr.addOnReconstructionCallback(&OnReconstructionCallback, nullptr);

    mgr.start();

    std::atomic<bool> terminate = false;
    std::thread checkKeyboardThread([&terminate]() {
        std::string line;
        std::getline(std::cin, line);
        terminate.store(true);
        std::cout << "Exiting ... please wait for application to close." << std::endl;
    });

    while (!terminate.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    checkKeyboardThread.join();
    mgr.stop();

    return 0;
}