#pragma once
#include <stdint.h>
#include <rpos/rpos_config.h>

namespace rpos { namespace features { namespace artifact_provider {
       
        enum RectangleAreaType
        {
            MaintenanceArea = 0,
            UnidirectionalSchedulingArea = 1,
            BidirectionalSchedulingArea = 2,
            HomeStackSchedulingArea = 3,
            //add new type here
            RectangleAreaTypeMax
        };

        enum DangerousAreaType
        {
            SlopeArea,
            NarrowCorridorArea
        };

        enum ElevatorDoorType : uint32_t
        {
            ElevatorFrontDoor = 0,
            ElevatorRearDoor = 1,
            ElevatorDoubleDoors = 2
        };

        static const char MetaDataKey_RectangleAreaType[] = "rectangle_area_type"; //for 2.8
        //fobidden area
        static const char MetaDataKey_EscapeDistance[] = "escape_distance";
        //elevator area
        static const char MetaDataKey_ElevatorID[] = "elevator_id";
        static const char MetaDataKey_ElevatorSillWidth[] = "elevator_sill_width";
        static const char MetaDataKey_ElevatorSchedulingPointDist[] = "elevator_scheduling_point_dist";
        static const char MetaDataKey_ElevatorDisableDefaultSchedulingPoint[] = "elevator_disable_default_scheduling_point";
        static const char MetaDataKey_ElevatorExtraFrontSchedulingPoints[] = "elevator_extra_front_scheduling_points";
        static const char MetaDataKey_ElevatorExtraRearSchedulingPoints[] = "elevator_extra_rear_scheduling_points";
        static const char MetaDataKey_ElevatorDoorType[] = "elevator_door_type";
        static const char MetaDataKey_ElevatorDepthValidHalfAngle[] = "elevator_depth_valid_half_angle";
        //dangerous area
        static const char MetaDataKey_DangerousAreaType[] = "dangerous_area_type";
        static const char MetaDataKey_MaxLineSpeed[] = "max_line_speed";
        // restricted area
        static const char MetaDataKey_RestrictedSchedulingPoints[] = "restricted_scheduling_points";
        static const char MetaDataKey_RestrictedRobotsNumberLimit[] = "restricted_robots_number_limit";
} } }
