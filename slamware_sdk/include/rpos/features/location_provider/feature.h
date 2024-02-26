/*
* feature.h
* Location Provider feature
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-05-25
* Copyright 2014~2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>

#include <rpos/core/feature.h>
#include <rpos/core/pose.h>
#include <rpos/features/location_provider/points_map.h>
#include <rpos/features/location_provider/rectangle_area_map.h>
#include <vector>
#include <boost/optional.hpp>

#include "map.h"

namespace rpos {
    namespace features {

        namespace detail {
            class LocationProviderImpl;
        }

        namespace location_provider {
            enum AuxLocalizationSource {
                AuxLocalizationSourceUwb
            };

            struct AuxLocalizationStatus {
                /**
                * @brief the source of the aux localization, currently only UWB is supported
                */
                AuxLocalizationSource source;

                /**
                * @brief is map update enabled
                */
                bool enableMapUpdate;

                /**
                * @brief is this localization source supported by the robot
                */
                bool isSupported;

                /**
                * @brief seconds since last update of this localization source, in seconds. -1 for never updated
                */
                int secondsSinceLastReport;
            };

            struct RobotFloorStatus
            {
                std::string building;
                std::string floor;
                boost::optional<std::string> elevatorIdRobotIn;
            };
        }

        class RPOS_CORE_API LocationProvider : public rpos::core::Feature{
        public:
            typedef detail::LocationProviderImpl impl_t;

            RPOS_OBJECT_CTORS_WITH_BASE(LocationProvider, rpos::core::Feature);
            LocationProvider(boost::shared_ptr<detail::LocationProviderImpl> impl);
            ~LocationProvider();

        public:
            std::vector<location_provider::MapType> getAvailableMaps();
            std::vector<location_provider::MapKind> getAvailableMapKinds();
            location_provider::Map getMap(location_provider::MapType type, core::RectangleF area, location_provider::MapKind kind);
            bool setMap(const location_provider::Map& map, location_provider::MapType type, location_provider::MapKind kind, bool partially);
            bool setMapAndPose(const core::Pose& pose, const location_provider::Map& map, const location_provider::MapType& type, const location_provider::MapKind& kind, bool partially);
            core::RectangleF getKnownArea(location_provider::MapType type, location_provider::MapKind kind);
            bool clearMap();
            bool clearMap(location_provider::MapKind kind);

            core::Location getLocation();
            core::Pose getPose();
            bool setPose(const core::Pose& pose);
            bool getMapLocalization();
            bool setMapLocalization(bool localization);
            bool getMapUpdate(location_provider::MapKind kind = location_provider::EXPLORERMAP);
            bool setMapUpdate(bool update, location_provider::MapKind kind = location_provider::EXPLORERMAP);
            bool getMapLoopClosure();
            bool setMapLoopClosure(bool loopClosure);
            int getLocalizationQuality();
            location_provider::PointPDF getAuxLocation();
            bool getHomePose(core::Pose&);
            bool setHomePose(core::Pose);

            std::vector<core::PoseEntry> getHomeDocks();
            bool setHomeDocks(const std::vector<core::PoseEntry>& docks); 
            //add home dock according to current robot pose
            core::PoseEntry registerHomeDock(const rpos::core::Metadata& metadata);
            //add home dock in the specified pose 
            core::PoseEntry addHomeDock(const core::PoseEntry& dock);
            bool editHomeDock(const core::PoseEntry& dock);
            bool eraseHomeDock(const std::string& id);
            bool clearHomeDocks();

            location_provider::AuxLocalizationStatus getAuxLocalizationStatus(location_provider::AuxLocalizationSource source);
            bool getImuInRobotCoordinate(rpos::core::Imu &);
            bool updateRobotFloorStatus(const location_provider::RobotFloorStatus& status);
            bool getRobotFloorStatus(location_provider::RobotFloorStatus& status);
            boost::optional<core::VisualTagPose> getTagPose();
            rpos::core::Location searchAttainableTarget(const rpos::core::Location &start, const rpos::core::Location& end, const float serachDistance);
            //get boundaries of corridor
            //@param minLength minimum length of the boundary
            //@param maxDistanceFromRobot if distance from robot to line is larger than maxDistanceFromRobot, ignore it
            std::vector<core::Line> getCorridorBoundaries(float minLength, float maxDistanceFromRobot);

        };
        
    }
}
