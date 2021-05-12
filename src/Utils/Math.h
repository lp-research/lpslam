#pragma once

#include "DataTypes/Space.h"

#include <boost/math/constants/constants.hpp>
#include <cmath>

namespace LpSlam {
namespace Math {

    inline double toRad(double deg) {
        return boost::math::constants::pi<double>() * (deg / 180.0);
    }

    inline double toDegree(double rad) {
        return (rad / boost::math::constants::pi<double>()) * 180.0;
    }

    inline Vector3 posSignVector(Vector3 v) {
        return Vector3( std::abs(v[0]),
            std::abs(v[1]),
            std::abs(v[2]));
    }
}
}
