#pragma once
#include <string>
#include <rpos/core/pose.h>

namespace rpos { namespace message { namespace schedule {

    struct NeighborRobotInfo  //for lora
    {
        std::string device_id;
        rpos::core::Pose pose;
        float robot_size;
        std::int64_t timestamp;
    };

}}}