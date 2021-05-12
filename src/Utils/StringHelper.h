#pragma once

#include <string>
#include <sstream>

#include "Utils/Math.h"
#include "DataTypes/Space.h"

namespace LpSlam{
    namespace strings {
        inline std::string orientationToString(Orientation const& o) {
            std::stringstream sout;
            sout << " w = " << o.value.w() << " x = " << o.value.x() << " y = " << o.value.y() << " z = " << o.value.z();
            sout << " sigma = " << Math::toDegree(o.sigma) << "*";
            return sout.str();
        }

        inline std::string positionToString(Position3 const& p) {
            std::stringstream sout;
            sout << " x = " << p.value.x() << " y = " << p.value.y() << " z = " << p.value.z();
            sout << " xs = " << p.sigma.x() << " ys = " << p.sigma.y() << " zs = " << p.sigma.z();
            return sout.str();
        }        

        inline std::string globalStateToString(GlobalState const& gs) {
            std::stringstream sout;
            sout << positionToString(gs.position) << " " << orientationToString(gs.orientation) << std::endl; 
            return sout.str();
        }

        inline std::string globalStateToString(GlobalStateInTime const& inTime) {
            std::stringstream sout;
            sout << " timestamp = " << inTime.first.system_time.time_since_epoch().count() << " " << globalStateToString(inTime.second);
            return sout.str();
        }

        template <class TStreamAble>
        inline std::string streamableToString(TStreamAble const& mat) {
            std::stringstream sout;
            sout << mat;
            return sout.str();
        }

        template<class TIntType>
        inline std::string integerToFileNamePadded(TIntType const& num,const size_t n_zero = 6) {
            std::string rawFileNumber = boost::lexical_cast<std::string>(num);

            // fill with leading zeros
            rawFileNumber = std::string(n_zero - rawFileNumber.length(), '0') + rawFileNumber;
            return rawFileNumber;
        }

    }
}