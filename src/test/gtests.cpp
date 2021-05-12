#include <gtest/gtest.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_sinks.h>

int main(int argc, char **argv) {
    spdlog::set_level(spdlog::level::info);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}