/**
* feature.h
* The Motion Planner feature
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/feature.h>
#include <rpos/core/pose.h>
#include <boost/optional.hpp>
#include <rpos/core/geometry.h>
#include "move_action.h"
#include "velocity_control_move_action.h"
#include "../system_resource/laser_scan.h"
#include <rpos/core/pose_entry.h>

namespace rpos {
    namespace features {

        namespace detail {
            class MotionPlannerImpl;
        }

        namespace motion_planner {

            enum NavigationMode
            {
                NavigationModeFree = 0,
                NavigationModeStrictVirtualTrack,
                NavigationModePriorityVirtualTrack,
            };

            enum MoveOptionFlag
            {
                MoveOptionFlagNone = 0,
                MoveOptionFlagAppending = 1,
                MoveOptionFlagMilestone = 2,                    //deprecated
                MoveOptionFlagNoSmooth = 4,
                MoveOptionFlagKeyPoints = 8,                    //deprecated
                MoveOptionFlagPrecise = 16,
                MoveOptionFlagWithYaw = 32,
                MoveOptionFlagReturnUnreachableDirectly = 64,
                MoveOptionFlagKeyPointsWithOA = 128,            //deprecated
                MoveOptionFlagWithFailRetryCount = 512,
                MoveOptionFlagFindPathIgnoringDynamicObstacles = 1024
            };

            struct RPOS_CORE_API MoveOptions
            {
                MoveOptions();
                MoveOptions(NavigationMode optionMode, MoveOptionFlag optionFlag);

                NavigationMode mode;
                MoveOptionFlag flag;
                boost::optional<double> speed_ratio;
                boost::optional<int> fail_retry_count;
                boost::optional<double> yaw;
                boost::optional<double> acceptable_precision;

                bool hasFlag(MoveOptionFlag flag);
                void addFlag(MoveOptionFlag flag);
            };

            enum RecoverLocalizationMovement
            {
                RecoverLocalizationMovementUnknown,
                RecoverLocalizationMovementNoMove,
                RecoverLocalizationMovementRotateOnly,
                RecoverLocalizationMovementAny,
                RecoverLocalizaitonMovementManual
            };

            struct RPOS_CORE_API RecoverLocalizationOptions
            {
                RecoverLocalizationOptions(boost::optional<int> ms = boost::none);
                boost::optional<int> maxRecoverTimeInMilliSeconds;
                boost::optional<RecoverLocalizationMovement> recoverMovementType;
                boost::optional<rpos::features::system_resource::LaserScan> observation;
            };

            enum GoHomeFlag {
                Dock,
                NoDock
            };

            struct RPOS_CORE_API GoHomeOptions
            {
                GoHomeOptions();
                GoHomeOptions(GoHomeFlag flag, bool enableBack, int retryCnt);

                GoHomeFlag flag;
                boost::optional<bool> enable_back_to_landing_point_after_fail;
                boost::optional<int> charging_base_fail_retry_count;
            };

            enum VelocityControlFlag {
                MonitoredByLocalizationQuality = 0,
                NotMonitored
                };

            enum ElevatorDoor {
                ElevatorDoorFront,
                ElevatorDoorRear
            };

            enum ElevatorStoppingYaw {
                FaceToFrontDoor,
                FaceToRearDoor
            };

            enum PoseToElevatorCalcFlag {
                PoseToElevatorCalcFlagWithPoint,
                PoseToElevatorCalcFlagWithBlock
            };

            enum PositionalRelationToElevator {
                UnKnown = 0,
                InElevator,
                NearElevatorSill,
                OutOfElevator
            };
            
            struct RPOS_CORE_API EnterElevatorActionOptions
            {
                EnterElevatorActionOptions();
                boost::optional<int> timeout_in_ms;
                motion_planner::ElevatorDoor elevator_door_flag;
                motion_planner::ElevatorStoppingYaw elevator_stopping_yaw;
                boost::optional<bool> use_conservative_mode;
                MoveOptionFlag move_flag;
            };

            struct RPOS_CORE_API LeaveElevatorActionOptions
            {
                LeaveElevatorActionOptions(); 
                boost::optional<int> timeout_in_ms;
                boost::optional<int> arrive_door_timeout_in_ms;
                boost::optional<int> search_path_timeout_in_ms;
                boost::optional<int> on_elevator_door_timeout_in_ms;
                motion_planner::ElevatorDoor elevator_door_flag;
                bool if_need_reach_milestone;
                MoveOptions move_options;
            };

            struct RPOS_CORE_API RotateActionOptions
            {
                RotateActionOptions();
                boost::optional<int> timeout_in_ms;
                bool if_rotate_circumcircle_collide;
                bool wait_after_can_rotate;
            };

            enum BackUpModeFromTag {
                BackUpFree = 0,
                BackUpInNarrowCorridor
            };

            struct RPOS_CORE_API BackFromTagOptions
            {
                BackFromTagOptions();
                BackUpModeFromTag back_mode;
                boost::optional<float> back_up_distance;
                boost::optional<float> back_up_angle;
                boost::optional<rpos::core::VisualTagPose> tag_pose;
            };

            enum CheckOccupiedKinds
            {
                CheckOccupiedKindsLocal = 1,
                CheckOccupiedKindsGlobal = 2,
            };
        }

        class RPOS_CORE_API MotionPlanner : public rpos::core::Feature{
        public:
            typedef detail::MotionPlannerImpl impl_t;

            RPOS_OBJECT_CTORS_WITH_BASE(MotionPlanner, rpos::core::Feature);
            MotionPlanner(boost::shared_ptr<detail::MotionPlannerImpl> impl);
            ~MotionPlanner();

        public:
            rpos::actions::MoveAction goHome(const rpos::features::motion_planner::GoHomeOptions& options = rpos::features::motion_planner::GoHomeOptions());
            rpos::actions::MoveAction moveTo(const std::vector<rpos::core::Location>& locations, bool appending);
            rpos::actions::MoveAction moveTo(const rpos::core::Location& location, bool appending); 
            rpos::actions::MoveAction moveTo(const std::vector<rpos::core::Location>& locations, const motion_planner::MoveOptions& options );
            rpos::actions::MoveAction moveTo(const rpos::core::Location& location, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction moveBy(const rpos::core::Direction& direction);
            rpos::actions::MoveAction moveBy(const rpos::core::Direction& direction, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction moveBy(float theta, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction rotateTo(const rpos::core::Rotation& orientation);
            rpos::actions::MoveAction rotateTo(const rpos::core::Rotation& orientation, const motion_planner::RotateActionOptions& options);
            rpos::actions::MoveAction rotate(const rpos::core::Rotation& rotation);
            rpos::actions::MoveAction rotate(const rpos::core::Rotation& rotation, const motion_planner::RotateActionOptions& options);
            rpos::actions::MoveAction recoverLocalization(const core::RectangleF& area, const motion_planner::RecoverLocalizationOptions& options);
            rpos::actions::MoveAction recoverLocalizationByDock();
            void recoverLocalizationByManual(const core::RectangleF& area, const motion_planner::RecoverLocalizationOptions& options);

            virtual rpos::actions::VelocityControlMoveAction velocityControl(rpos::features::motion_planner::VelocityControlFlag flag = rpos::features::motion_planner::MonitoredByLocalizationQuality);
            rpos::actions::MoveAction getCurrentAction();
            rpos::features::motion_planner::Path searchPath(const rpos::core::Location& location, int timeoutMs);
            std::vector<std::string> enumSupportedMotionStrategies();
            bool setMotionStrategy(const std::string& strategy);
            std::string getCurrentMotionStrategy();
            std::vector<std::string> getElevatorIDList();
            //vector[0]: center scheduling pose, others are additional scheduling poses
            std::vector<rpos::core::Pose> getElevatorSchedulingPoses(const std::string& elevatorID, motion_planner::ElevatorDoor doorFlag = motion_planner::ElevatorDoorFront);
            rpos::features::motion_planner::PositionalRelationToElevator getPoseRelationToElevator(const std::string& elevatorID, motion_planner::PoseToElevatorCalcFlag poseCalcFlag);
            rpos::actions::MoveAction enterElevator(const std::string& elevatorID);
            rpos::actions::MoveAction enterElevator(const std::string& elevatorID, const features::motion_planner::EnterElevatorActionOptions& options);
            rpos::actions::MoveAction leaveElevator(const std::string& elevatorID, const core::Location& location);
            rpos::actions::MoveAction leaveElevator(const std::string& elevatorID, const core::Location& location, const features::motion_planner::LeaveElevatorActionOptions& options);
            rpos::actions::MoveAction moveToTag(const rpos::core::VisualTagPose& tagPose, const boost::optional<rpos::core::Pose>& targetPoseToTag);
            rpos::actions::MoveAction backUpAfterMoveToTag(const features::motion_planner::BackFromTagOptions &options);
            std::vector<rpos::core::Location> doMultTaskDispatch(boost::optional<rpos::core::Location> & startPoint, std::vector<rpos::core::Location> passPoints,boost::optional<rpos::core::Location>& endPoint);
            bool searchElevatorPath(const std::string& elevatorID, int timeoutInMs);
            bool checkOccupied(const rpos::core::Pose& pose, rpos::features::motion_planner::CheckOccupiedKinds kinds);
        };

    }
}
