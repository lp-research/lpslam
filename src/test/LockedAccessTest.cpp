
#include <gtest/gtest.h>

#include <spdlog/spdlog.h>

#include "Utils/LockedAccess.h"

using namespace LpSlam;

TEST(locked_access, test)
{
    LockedAccess<std::string> safeObject;

    {
        auto access = safeObject.get();
        access.payload() = "Hallo";
    }

    {
        auto access = safeObject.get();
        ASSERT_EQ("Hallo", access.payload());
    }
}