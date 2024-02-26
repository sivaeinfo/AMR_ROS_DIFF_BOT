#pragma once

#include <rpos/core/pose.h>
#include <rpos/system/types.h>
#include <rpos/core/metadata.h>
#include <rpos/robot_platforms/objects/map_layer.h>
#include <queue>
#include <vector>

namespace rpos { namespace robot_platforms { namespace objects {

    struct polygonArea
    {
        rpos::core::SegmentID id;
        rpos::system::types::_u8 count;
        std::deque<core::Location> locations;
        rpos::core::Metadata metadata;
    };

    class RPOS_SLAMWARE_API PolygonAreaMapLayer : public MapLayer
    {
    public:
        static const char* const Type;

        PolygonAreaMapLayer();
        virtual ~PolygonAreaMapLayer();

    public:
        virtual void clear(void);

    public:
        const std::vector<polygonArea> & areas() const;
        std::vector<polygonArea> & areas();

    private:
        std::vector<polygonArea> areas_;
    };

}}}
