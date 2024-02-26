#pragma once
#include <rpos/core/rpos_core_config.h>
#include <rpos/core/metadata.h>
#include <rpos/core/pose.h>
#include <rpos/system/types.h>
#include <map>

namespace rpos { namespace core { 
    struct RPOS_CORE_API PoseEntry
    {
        std::string name;
        std::vector<std::string> tags;
        core::Pose pose;
        rpos::system::types::_u8 flags;
        Metadata metadata;

        bool operator==(const PoseEntry& that) const
        {
            return (name == that.name && tags == that.tags && pose == that.pose && flags == that.flags && metadata == that.metadata);
        }
    };

    struct RPOS_CORE_API VisualTagPose
    {
        std::vector<int32_t> tagIds;
        core::Pose pose;
    };

    typedef std::map<std::string, PoseEntry> PoseEntryMap;
    typedef std::pair<std::string, PoseEntry> PoseEntryPair;
}}