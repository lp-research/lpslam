#pragma once

#include "DataTypes/Space.h"

#include <tbb/concurrent_queue.h>

namespace LpSlam {

    struct ResultQueueEntry {

        ResultQueueEntry() = default;

        ResultQueueEntry( GlobalStateInTime const& gs ) : globalStateInTime(gs) {
        }

        GlobalStateInTime globalStateInTime;

        // if exitSignal = true, the thread waiting for new entries
        // can exit
        bool exitSignal = false;
    };

    typedef tbb::concurrent_bounded_queue<ResultQueueEntry> ResultQueue;

}