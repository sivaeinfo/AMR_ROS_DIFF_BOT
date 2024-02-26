#pragma once  
#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <json/json.h>
#include <string>
#include <vector>
#include <cstdint>

namespace rpos { namespace features { namespace system_resource {

    enum PrivateConfigType{
        PrivateConfigTypeUnknown = 0
        , LidarExtrinsicParam
        , MotionStrategy
        , CustomizedLidarConfig
        , TagCameraIntrinsic
    };

    struct SlamwarePrivateConfig {
        PrivateConfigType type;
        std::string id;
        Json::Value config;
    };

} } }
 