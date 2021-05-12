#pragma once

#include <fstream>

namespace LpSlam {
namespace file_util {

inline bool does_exist(std::string const& fname)
{
    std::ifstream infile(fname);
    return infile.good();
}

}
}

/*
disabled on non-Visual Studio build because not supported by gcc-8 libstdc++
*/

#include <vector>
#include <filesystem>
#include <regex>

namespace LpSlam {
namespace FileSystem {
    std::vector<std::filesystem::path> imagesInFolder(std::filesystem::path folderpath) {
        std::vector<std::filesystem::path> images;

        for(auto& p: std::filesystem::directory_iterator(folderpath)) {
            if (!p.is_regular_file())
                continue;

            auto ext = p.path().extension();
            if ((ext == ".jpg" ) || (ext == ".png" )) {
                images.push_back(p);
            }
        }

        std::sort(images.begin(), images.end());
        return images;
    }
}
}
