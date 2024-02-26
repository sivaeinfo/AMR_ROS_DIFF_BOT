#pragma once
#include <cstdint>
#include <unordered_map>
#include <boost/chrono.hpp>

namespace rpos { namespace system { namespace util {
    class DurationWatch {
    public:
        DurationWatch();
        ~DurationWatch();
    public:
        void reset();
        void start(const std::string& label);
        boost::chrono::steady_clock::duration record(const std::string& label);
        std::uint32_t totalInMs();
        std::string toString();
    protected:
        boost::chrono::steady_clock::time_point start_;
        std::unordered_map<std::string, boost::chrono::steady_clock::time_point> starts_;
        std::unordered_map<std::string, boost::chrono::steady_clock::duration> durations_;
    };

}}}
