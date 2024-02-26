#pragma once

#include <rpos/robot_platforms/slamware_sdp_platform_config.h>
#include <rpos/system/exception.h>
#include <rpos/core/detail/geometry_matrix.h>

#if (defined(_DEBUG) || defined(DEBUG))
    #define RPOS_COMPOSITEMAP_ASSERT              assert
#else
    #define RPOS_COMPOSITEMAP_ASSERT(expr)
#endif
#define RPOS_COMPOSITEMAP_NOT_USED(x)       ((void)(x))

#define RPOS_COMPOSITEMAP_THROW_EXCEPTION(msg)      throw ::rpos::robot_platforms::objects::CompositeMapException((msg), __FILE__, __FUNCTION__, __LINE__)

#define RPOS_COMPOSITEMAP_USAGE_EXPLORE                         "explore"
#define RPOS_COMPOSITEMAP_USAGE_SWEEP                           "sweep"
#define RPOS_COMPOSITEMAP_USAGE_VIRTUAL_WALL                    "virtual_walls"
#define RPOS_COMPOSITEMAP_USAGE_VIRTUAL_TRACK                   "virtual_tracks"
#define RPOS_COMPOSITEMAP_USAGE_POINTS_MAP                      "pointsmap"
#define RPOS_COMPOSITEMAP_USAGE_RECTANGLE_AREA                  "rectangle_area"
#define RPOS_COMPOSITEMAP_USAGE_HOME_DOCK_POSE                  "home_dock_pose"
#define RPOS_COMPOSITEMAP_USAGE_POI                             "point_of_interest"
#define RPOS_COMPOSITEMAP_USAGE_DBOW                            "dbow"
#define RPOS_COMPOSITEMAP_USAGE_MARKER                          "marker"

#define RPOS_COMPOSITEMAP_METADATA_KEY_NAME                     "name"
#define RPOS_COMPOSITEMAP_METADATA_KEY_USAGE                    "usage"
#define RPOS_COMPOSITEMAP_METADATA_KEY_TYPE                     "type"
#define RPOS_COMPOSITEMAP_METADATA_KEY_COMPRESSION              "compression"
#define RPOS_COMPOSITEMAP_METADATA_KEY_COUNT                    "count"

#define RPOS_COMPOSITEMAP_METADATA_KEY_ORIGIN_X                 "origin_x"
#define RPOS_COMPOSITEMAP_METADATA_KEY_ORIGIN_Y                 "origin_y"
#define RPOS_COMPOSITEMAP_METADATA_KEY_DIMENSION_WIDTH          "dimension_width"
#define RPOS_COMPOSITEMAP_METADATA_KEY_DIMENSION_HEIGHT         "dimension_height"
#define RPOS_COMPOSITEMAP_METADATA_KEY_RESOLUTION_X             "resolution_x"
#define RPOS_COMPOSITEMAP_METADATA_KEY_RESOLUTION_Y             "resolution_y"

#define RPOS_COMPOSITEMAP_METADATA_KEY_FEATURE_TYPE             "feature_type"
#define RPOS_COMPOSITEMAP_METADATA_KEY_ID                       "id"
#define RPOS_COMPOSITEMAP_METADATA_KEY_VERSION                  "version"
#define RPOS_COMPOSITEMAP_METADATA_KEY_TRANSFORM                "transform"
#define RPOS_COMPOSITEMAP_METADATA_KEY_FLOOR                    "floor"
#define RPOS_COMPOSITEMAP_METADATA_KEY_BUILDING                 "building"
#define RPOS_COMPOSITEMAP_METADATA_KEY_ORDER                    "order"
#define RPOS_COMPOSITEMAP_METADATA_KEY_DEFAULT                  "default"

namespace rpos { namespace robot_platforms { namespace objects {

    class RPOS_SLAMWARE_API CompositeMapException : public rpos::system::detail::ExceptionBase
    {
    private:
        typedef rpos::system::detail::ExceptionBase             super_class_type;
    public:
        explicit CompositeMapException(const std::string& msg = "")
            : super_class_type(msg)
        {
            //
        }
        CompositeMapException(const std::string& msg, const char* file, const char* func, int line)
            : super_class_type(msg)
        {
            this->super_class_type::Init(file, func, line);
        }
        virtual ~CompositeMapException(void)
        {
            //
        }
    };

    struct MapDescription
    { 
        std::string filePath;
        std::string floor;
        std::string building; 
        rpos::core::Vector3f transform; 
        bool isDefaultFloor;
        MapDescription(): isDefaultFloor(false){}
    };
     
}}}
