#ifndef PROTO_STREAM_H
#define PROTO_STREAM_H

#include <fstream>
#include <iterator>
#include <vector>
#include <cstdint>
#include <exception>

#include <spdlog/spdlog.h>

class ProtoStream {
public:
    template < class TMessage, class TMessageType >
    bool toStream(TMessageType msgType, TMessage const& m, std::ofstream * outstream) {
        size_t size = m.ByteSizeLong();
        ensureMsgSize(size);

        if (!m.SerializeToArray(m_buffer.data(), size))
            return false;

        uint64_t uint_msgType = msgType;
        uint64_t uint_size = size;

        outstream->write(reinterpret_cast<char*>(&uint_msgType), sizeof(uint64_t));
        outstream->write(reinterpret_cast<char*>(&uint_size), sizeof(uint64_t));

        std::ostream_iterator<char> output_iterator(*outstream);
        std::copy(m_buffer.begin(), m_buffer.begin() + size, output_iterator);

        return true;
    }

    template < class TMessageType>
    bool fromStreamNextMessageType(std::ifstream * instream, TMessageType & msgType ) {
        uint64_t intMsgType;

        instream->read(reinterpret_cast<char*>(&intMsgType), sizeof(uint64_t));
        
        msgType = static_cast<TMessageType>(intMsgType);

        return true;
    }

    template < class TMessage >
    bool fromStreamNextMessage(std::ifstream * instream, TMessage & m) {
        uint64_t intMsgSize;
        instream->read(reinterpret_cast<char*>(&intMsgSize), sizeof(uint64_t));
        try {
            ensureMsgSize(intMsgSize);
        }
        catch (std::invalid_argument const&) {
            // can happen if very large buffer or corrupt butter
            return false;
        }
        instream->read(m_buffer.data(), intMsgSize);

        return m.ParseFromArray(m_buffer.data(), intMsgSize);
    }

private:

    void ensureMsgSize(size_t size) {
        if (size > m_maxBufferSize) {
            spdlog::critical("Requested buffer size {0} is larger than buffer limit {1}",
                size, m_maxBufferSize);
            throw std::invalid_argument("Requested buffer size is larger than buffer limit.");
        }
        if (m_buffer.size() < size) {
            m_buffer.resize(size);
    }
    }

    std::vector<char > m_buffer;
    // up to 5 MB to be able to hold pictures
    const size_t m_maxBufferSize = 5000000;
};

#endif