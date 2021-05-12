#pragma once

#include "DataTypes/Space.h"

#include <boost/noncopyable.hpp>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <optional>
#include <numeric>

#include <spdlog/spdlog.h>


namespace LpSlam {

// this Controller assumes all values provided to computeControl are equally spaced in time
class PidController {
public:
    PidController(double Kp, double Ki, double Kd) : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd) {

    }

    double computeControl(double setpoint, double processpoint) {
        const double e = setpoint - processpoint;

        double diff_e = 0.0;
        double int_e = 0.0;

        if (m_lastError.has_value()) {
            diff_e = e - m_lastError.value();
            int_e = (e + m_lastError.value()) / 2.0;
        }

        m_lastError = e;

        return m_Kp * e + m_Ki * int_e + m_Kd * diff_e;
    }

private:
    double m_Kp = 1.0;
    double m_Ki = 1.0;
    double m_Kd = 1.0;

    std::optional<double> m_lastError;

    std::vector<double> m_lastEs;
};

}
