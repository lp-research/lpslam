#pragma once

#include <mutex>

namespace LpSlam {

template <class TPayload>
class LockedAccess {
public:

    class AccessToken {
    private:
        friend LockedAccess<TPayload>;
        AccessToken(std::mutex & mt, TPayload & pl) :
            m_lock(mt),
            m_payload(pl) {
            }
    public:
        TPayload & payload() {
            return m_payload;
        }

    private:
        std::lock_guard<std::mutex> m_lock;
        TPayload & m_payload;
    };

    AccessToken get() {
        // wait till get get a lock to hand it out !
        return AccessToken(m_payloadMutex, m_payload);
    }

private:
    std::mutex m_payloadMutex;
    TPayload m_payload;
};

}
