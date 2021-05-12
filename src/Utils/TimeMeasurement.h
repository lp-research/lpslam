#pragma once

#include "DataTypes/Space.h"

#include <boost/noncopyable.hpp>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <optional>
#include <numeric>

#include <spdlog/spdlog.h>

namespace chr = std::chrono;

namespace LpSlam {

inline double durationToSeconds(chr::high_resolution_clock::duration dur) {
	std::chrono::nanoseconds nsecs = chr::duration_cast<
		std::chrono::nanoseconds>(dur);
	return nsecs.count() * std::pow(10, -9);
}

inline double timestampToSeconds(TimeStamp first_timestamp, TimeStamp this_timestamp) {
        // convert to seconds
    auto imgTimestamp = std::chrono::duration<double, std::chrono::seconds::period>(
        this_timestamp - first_timestamp
    );

    return imgTimestamp.count();
}

/**
 * Base class for measurng timing differences
 */
class TimingBase: boost::noncopyable {
public:
	/**
	 * Create and start the time taking
	 */
	TimingBase() {
		start();
	}

    TimingBase(std::string const& topic) : m_topic(topic) {
        start();
    }

	/**
	 * Take the start time
	 */
	void start() {
		m_startTime = chr::high_resolution_clock::now();
	}

	/**
	 * return the time difference in seconds between the calls to start() and
	 * end()
	 */
	float delta() const {
		std::chrono::milliseconds secs = chr::duration_cast<
				std::chrono::milliseconds>(m_endTime - m_startTime);
		return secs.count() * 0.001f;
	}

	/**
	 * Take the stop time
	 */
	float end() {
		m_endTime = chr::high_resolution_clock::now();
		return delta();
	}

    void report() {
        end();
        spdlog::info("Topic {0} took {1} seconsd", m_topic, delta());
    }

private:
	/**
	 * Time point when start() was called
	 */
	chr::high_resolution_clock::time_point m_startTime;

	/**
	 * Time point when end() was called
	 */
	chr::high_resolution_clock::time_point m_endTime;

    std::string m_topic;
};

class FramerateCompute {
public:
	void gotFrame() {
		const auto now = std::chrono::high_resolution_clock::now();
		if (m_lastTimestamp) {
			const auto toLastFrame = durationToSeconds(now - m_lastTimestamp.value());
			m_frametimes.push_back(toLastFrame);
		}
		m_lastTimestamp = now;
	}

	std::optional<double> getFramerate() {
		if (m_frametimes.size() > 2) {
			double avg_frametime = std::accumulate(m_frametimes.begin(), m_frametimes.end(), 0.0) / double(m_frametimes.size());
			return 1.0 / avg_frametime;
		} else {
			return std::nullopt;
		}
	}

private:
	boost::circular_buffer<double> m_frametimes = boost::circular_buffer<double>(10);
	std::optional<std::chrono::high_resolution_clock::time_point> m_lastTimestamp;
};

}
