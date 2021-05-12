
#include "Manager/SlamManager.h"
#include "Interface/LpSlamTypes.h"
#include <gtest/gtest.h>

#include <string>
#include <sstream>

using namespace LpSlam;

// make sure providing wrong json config is handled
// properly
TEST(slam_manager, read_config_file)
{
    const std::string testJsonConfigFile = "lpgf_unitttest_config_file.json";

    {
        // no config provided
        SlamManager fm;

        ASSERT_FALSE(fm.readConfigurationFile("does_not_exist.json"));
    }

    {
        SlamManager fm;

        nlohmann::json j_test_datasource = {
        {"datasources",  { {
            { "type", "FileSource" }}
           }
        } };

        {
            std::ofstream o(testJsonConfigFile);
            o << std::setw(4) << j_test_datasource << std::endl;
            o.close();
        }

        ASSERT_TRUE(fm.readConfigurationFile(testJsonConfigFile));
    }

    {
        SlamManager fm;
        {
            std::ofstream o(testJsonConfigFile);
            o << std::setw(4) << "datasources {{"  << std::endl;
            o.close();
        }

        // syntax error in Json
        ASSERT_FALSE(fm.readConfigurationFile(testJsonConfigFile));
    }

    {
        SlamManager fm;

        nlohmann::json j_test_datasource = {
        {"datasources",  { {
            { "type", "FileSource" },
            { "configuration",
                {
                    { "file_names" , "blah"}
                }
            }
           }
        }
        } };

        {
            std::ofstream o(testJsonConfigFile);
            o << std::setw(4) << j_test_datasource << std::endl;
            o.close();
        }
        ASSERT_TRUE(fm.readConfigurationFile(testJsonConfigFile));
    }
}


TEST(slam_manager, read_camera_calibration)
{
    const std::string testJsonConfigFile = "lpgf_unitttest_config_file.json";
    {
        SlamManager fm;

        nlohmann::json j_test_datasource = {
        {"cameras", {
        {
            { "number", 0 },
            { "model", "fisheye" },
            { "fx", 1.0},
            { "fy", 2.0},
            { "cx", 3.0},
            { "cy", 4.0},
            { "distortion", {10.0, 11.0, 12.0, 13.0 }},
            { "resolution_x", 320},
            { "resolution_y", 240}
        },
        {
            { "number", 1 },
            { "model", "perspective" },
            { "fx", 21.0},
            { "fy", 22.0},
            { "cx", 23.0},
            { "cy", 24.0},
            { "distortion", {110.0, 111.0, 112.0, 113.0, 114.0}},
            { "resolution_x", 320},
            { "resolution_y", 240}
        }
        }
        }};

        {
            std::ofstream o(testJsonConfigFile);
            o << std::setw(4) << j_test_datasource << std::endl;
            o.close();
        }
        ASSERT_TRUE(fm.readConfigurationFile(testJsonConfigFile));

        ASSERT_TRUE(fm.getCameraConfiguration(0).has_value());
        ASSERT_TRUE(fm.getCameraConfiguration(1).has_value());
        auto cam0 = fm.getCameraConfiguration(0).value();
        auto cam1 = fm.getCameraConfiguration(1).value();

        ASSERT_EQ(cam0.distortion_function, LpSlamCameraDistortionFunction_Fisheye);
        ASSERT_NEAR(cam0.f_x, 1.0, 0.001);
        ASSERT_NEAR(cam0.f_y, 2.0, 0.001);
        ASSERT_NEAR(cam0.c_x, 3.0, 0.001);
        ASSERT_NEAR(cam0.c_y, 4.0, 0.001);

        ASSERT_NEAR(cam0.dist[0], 10.0, 0.001);
        ASSERT_NEAR(cam0.dist[1], 11.0, 0.001);
        ASSERT_NEAR(cam0.dist[2], 12.0, 0.001);
        ASSERT_NEAR(cam0.dist[3], 13.0, 0.001);

        ASSERT_EQ(cam1.distortion_function, LpSlamCameraDistortionFunction_Pinhole);
        ASSERT_NEAR(cam1.f_x, 21.0, 0.001);
        ASSERT_NEAR(cam1.f_y, 22.0, 0.001);
        ASSERT_NEAR(cam1.c_x, 23.0, 0.001);
        ASSERT_NEAR(cam1.c_y, 24.0, 0.001);

        ASSERT_NEAR(cam1.dist[0], 110.0, 0.001);
        ASSERT_NEAR(cam1.dist[1], 111.0, 0.001);
        ASSERT_NEAR(cam1.dist[2], 112.0, 0.001);
        ASSERT_NEAR(cam1.dist[3], 113.0, 0.001);
        ASSERT_NEAR(cam1.dist[4], 114.0, 0.001);
    }

    // too many parameters in distortion
    {
        SlamManager fm;

        nlohmann::json j_test_datasource = {
        {"cameras", {
        {
            { "number", 0 },
            { "model", "fisheye" },
            { "fx", 1.0},
            { "fy", 2.0},
            { "cx", 3.0},
            { "cy", 4.0},
            { "distortion", {10.0, 11.0, 12.0, 13.0, 34.0, 23.02, 2.3 }},
        }
        }}};

        {
            std::ofstream o(testJsonConfigFile);
            o << std::setw(4) << j_test_datasource << std::endl;
            o.close();
        }
        ASSERT_FALSE(fm.readConfigurationFile(testJsonConfigFile));
    }

    // unknown camera model
    {
        SlamManager fm;

        nlohmann::json j_test_datasource = {
        {"cameras", {
        {
            { "number", 0 },
            { "model", "what_!?" },
            { "fx", 1.0},
            { "fy", 2.0},
            { "cx", 3.0},
            { "cy", 4.0},
            { "distortion", {10.0, 11.0, 12.0}},
        }
        }}};

        {
            std::ofstream o(testJsonConfigFile);
            o << std::setw(4) << j_test_datasource << std::endl;
            o.close();
        }
        ASSERT_FALSE(fm.readConfigurationFile(testJsonConfigFile));
    }

}