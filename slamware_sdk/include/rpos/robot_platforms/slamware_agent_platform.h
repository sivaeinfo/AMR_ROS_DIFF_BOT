/**
* slamware_agent_platform.h
* Slamtec Slamware(r) Agent platform
*
* Created By Gabriel He @ 2016-01-28
* Copyright (c) 2016 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/features/artifact_provider.h>
#include <rpos/features/location_provider.h>
#include <rpos/features/motion_planner.h>
#include <rpos/features/system_resource.h>
#include <rpos/features/impact_sensor_feature.h>
#include <rpos/features/statistics_provider.h>
#include <rpos/robot_platforms/objects/composite_map.h>
#include <rpos/robot_platforms/objects/slamware_scheduler_service.h>
#include <rpos/robot_platforms/objects/slamware_firmware_service.h>
#include <rpos/robot_platforms/objects/pose_map_layer.h>
#include <rpos/robot_platforms/slamware_http_exception.h>
#include <rpos/core/parameter.h>
#include <boost/shared_ptr.hpp>

#include <vector>

namespace rpos { namespace robot_platforms {


    namespace detail {
        class SlamwareAgentPlatformImpl;
        class SlamwareActionFactory;
        class SlamwareHttpsClient;
    }

    using namespace robot_platforms::http;

    class RPOS_SLAMWARE_API SlamwareAgentPlatform
    {
        friend class detail::SlamwareActionFactory;
    public:
        typedef detail::SlamwareAgentPlatformImpl impl_t;

        SlamwareAgentPlatform();
        SlamwareAgentPlatform(boost::shared_ptr<impl_t> impl);
        ~SlamwareAgentPlatform();

    public:
        static SlamwareAgentPlatform connect(const std::string& host, int port);
        void disconnect();

    public:
        features::ArtifactProvider getArtifactProvider();
        features::LocationProvider getLocationProvider();
        features::MotionPlanner getMotionPlanner();
        features::SystemResource getSystemResource();
        features::ImpactSensor getImpactSensor();
        features::StatisticsProvider getStatisticsProvider();

    public:
        // Artifacts Provider APIs
        /// Slamwarecore related APIs
        std::vector <features::artifact_provider::RectangleArea> getRectangleAreas(features::artifact_provider::ArtifactUsage usage);
        
        bool addRectangleArea(features::artifact_provider::ArtifactUsage usage, const features::artifact_provider::RectangleArea& area);
        
        bool addRectangleAreas(features::artifact_provider::ArtifactUsage usage, const std::vector<features::artifact_provider::RectangleArea>& areas);
        
        bool removeRectangleAreaByIds(features::artifact_provider::ArtifactUsage usage, const std::vector<core::SegmentID>& ids);
        
        bool clearRectangleAreas(features::artifact_provider::ArtifactUsage usage); 

        std::vector<core::Line> getLines(features::artifact_provider::ArtifactUsage usage);

        bool addLine(features::artifact_provider::ArtifactUsage usage, const core::Line& line);

        bool addLines(features::artifact_provider::ArtifactUsage usage, const std::vector<core::Line>& lines);

        bool removeLineById(features::artifact_provider::ArtifactUsage usage, rpos::core::SegmentID id);

        bool clearLines(features::artifact_provider::ArtifactUsage usage);

        bool moveLine(features::artifact_provider::ArtifactUsage usage, const core::Line& line);
        
        bool moveLines(features::artifact_provider::ArtifactUsage usage, const std::vector<core::Line>& lines);

        std::vector<core::Line> getWalls();

        bool addWall(const core::Line& wall);

        bool addWalls(const std::vector<core::Line>& walls);

        bool clearWallById(const core::SegmentID& id);

        bool clearWalls();

        std::vector<core::RectangleF> getMapDiscrepancyMonitorAreas();

        bool addMapDiscrepancyMonitorArea(const core::RectangleF& area);
        
        bool addMapDiscrepancyMonitorAreas(const std::vector<core::RectangleF>& areas);
        
        bool removeMapDiscrepancyMonitorAreas(const std::vector<core::RectangleF>& areas);
        
        bool clearMapDiscrepancyMonitorAreas();

        robot_platforms::objects::PoseEntryMap getPOIs();

        bool setPOIs(const robot_platforms::objects::PoseEntryMap& pois);

        bool addPOI(const core::PoseEntry& poi);
        
        core::PoseEntry addPOIOnCurrentPose(const std::string& name, const core::Metadata& metadata);

        std::pair<std::string, core::PoseEntry> queryPOI(const std::string& name);

        std::pair<std::string, core::PoseEntry> queryClosestPOI(const rpos::core::Pose& pose);

        bool erasePOI(const std::string& name);

        bool clearPOIs();

    public:
        // Location Provider APIs
        /// Slamwarecore related APIs
        std::vector<features::location_provider::MapType> getAvailableMaps();
        
        std::vector<features::location_provider::MapKind> getAvailableMapKinds();

        features::location_provider::Map getMap(features::location_provider::MapType type, core::RectangleF area, features::location_provider::MapKind kind);

        bool setMapAndPose(const core::Pose& pose, const features::location_provider::Map& map, const features::location_provider::MapType& type, const features::location_provider::MapKind& kind, bool partially = false);
        
        bool setMap(const features::location_provider::Map& map, features::location_provider::MapType type, features::location_provider::MapKind kind, bool partially = false);

        core::RectangleF getKnownArea(features::location_provider::MapType type, features::location_provider::MapKind kind);

        bool clearMap();

        bool clearMap(features::location_provider::MapKind kind);

        core::Location getLocation();

        core::Pose getPose();

        bool setPose(const core::Pose& pose);

        bool getMapLocalization();

        bool setMapLocalization(bool localization);

        bool getMapUpdate(rpos::features::location_provider::MapKind kind = rpos::features::location_provider::EXPLORERMAP);

        bool setMapUpdate(bool update, rpos::features::location_provider::MapKind kind = rpos::features::location_provider::EXPLORERMAP);

        int getLocalizationQuality();

        features::location_provider::PointPDF getAuxLocation();

        bool getHomePose(core::Pose&);

        bool setHomePose(core::Pose);
        
        bool getImuInRobotCoordinate(rpos::core::Imu &); 

        boost::optional<core::VisualTagPose> getTagPose();

        rpos::core::Location searchAttainableTarget(const rpos::core::Location& start, const rpos::core::Location& end, const float serachDistance);
    
        std::vector<rpos::core::PoseEntry> getHomeDocks();
        bool setHomeDocks(const std::vector<rpos::core::PoseEntry>& docks);
        rpos::core::PoseEntry registerHomeDock(const rpos::core::Metadata& metadata);
        rpos::core::PoseEntry addHomeDock(const rpos::core::PoseEntry& dock);
        bool editHomeDock(const rpos::core::PoseEntry& dock);
        bool eraseHomeDock(const std::string& id);
        bool clearHomeDocks();

    public:
        // Motion Planner APIs
        /// Slamwarecore related APIs
        actions::MoveAction moveTo(const std::vector<core::Location>& locations, bool appending);

        actions::MoveAction moveTo(const core::Location& location, bool appending);

        actions::MoveAction moveTo(const std::vector<core::Location>& locations, const features::motion_planner::MoveOptions& options);

        actions::MoveAction moveTo(const core::Location& location, const features::motion_planner::MoveOptions& options);

        actions::MoveAction moveBy(const core::Direction& direction);

        actions::MoveAction moveBy(const core::Direction& direction, const features::motion_planner::MoveOptions& options);

        actions::MoveAction moveBy(float theta, const features::motion_planner::MoveOptions& options);

        actions::MoveAction rotateTo(const core::Rotation& orientation);

        actions::MoveAction rotateTo(const core::Rotation& orientation, const features::motion_planner::RotateActionOptions& options);

        actions::MoveAction rotate(const core::Rotation& rotation);

        actions::MoveAction rotate(const core::Rotation& rotation, const features::motion_planner::RotateActionOptions& options);

        actions::MoveAction recoverLocalization(const core::RectangleF& area, const features::motion_planner::RecoverLocalizationOptions& options=features::motion_planner::RecoverLocalizationOptions());
        
        actions::MoveAction recoverLocalizationByDock();

        rpos::actions::VelocityControlMoveAction velocityControl(rpos::features::motion_planner::VelocityControlFlag flag = rpos::features::motion_planner::MonitoredByLocalizationQuality);

        actions::MoveAction getCurrentAction();

        features::motion_planner::Path searchPath(const core::Location& location, int timeoutMs);

        actions::MoveAction goHome(const rpos::features::motion_planner::GoHomeOptions& options = rpos::features::motion_planner::GoHomeOptions());

        std::vector<rpos::core::Location> doMultTaskDispatch(boost::optional<rpos::core::Location>& startPoint, std::vector<rpos::core::Location> passPoints, boost::optional<rpos::core::Location>& endPoint);

    public:
        // System Resource APIs
        /// Base related APIs
        int getBatteryPercentage();

        bool getBatteryIsCharging();

        bool getDCIsConnected();

        features::system_resource::PowerStatus getPowerStatus();

        int getBoardTemperature();

        bool updateBinaryConfig(const Json::Value& jsnCfg);

        features::system_resource::DeviceInfo getDeviceInfo();

        int sendAndRecvUserDefinedCBUSMessage(const void* payload, const size_t payloadsize, std::vector<std::uint8_t>& recvDat);

        /// Slamwarecore related APIs
        void wakeUp();

        void hibernate();

        std::string getSDPVersion();

        std::string getSDKVersion();

        features::system_resource::LaserScan getLaserScan();

        bool restartModule(features::system_resource::RestartMode mode = features::system_resource::RestartModeSoft);

        bool setSystemParameter(const std::string& param, const std::string& value);

        std::string getSystemParameter(const std::string& param);

        bool shutdownSlamcore(const rpos::core::SlamcoreShutdownParam& shutdownArg);

        features::system_resource::BaseHealthInfo getRobotHealth();

        void clearRobotHealth(int errorCode);

        bool configurateNetwork(features::system_resource::NetworkMode mode, const std::map<std::string, std::string>& options);

        std::map<std::string, std::string> getNetworkStatus();

        features::system_resource::HeartBeatToken startHeartBeat(int heartBeatTimeoutInSeconds);

        void refreshHeartBeat(features::system_resource::HeartBeatToken token);

        void stopHeartBeat(features::system_resource::HeartBeatToken token);

        std::vector<features::system_resource::OperationAuditLog> getOperationAuditLogs();
        float getDynamicObstacleDistance();

    public:
        // Impact Sensor APIs
        /// Base related APIs
        bool getSensors(std::vector<features::impact_sensor::ImpactSensorInfo>& sensors);

        bool getSensorValues(std::map<features::impact_sensor::impact_sensor_id_t, features::impact_sensor::ImpactSensorValue>& values);

        bool getSensorValues(const std::vector<features::impact_sensor::impact_sensor_id_t>& sensorIds, std::vector<features::impact_sensor::ImpactSensorValue>& values);

        bool getSensorValue(features::impact_sensor::impact_sensor_id_t sensorId, features::impact_sensor::ImpactSensorValue& value);

    public:
        /// Slamwarecore related APIs
        robot_platforms::objects::CompositeMap getCompositeMap();

        bool setCompositeMap(const robot_platforms::objects::CompositeMap& map, const core::Pose& pose);

        bool setCompositeMap(const robot_platforms::objects::CompositeMap& map, const core::Pose& pose, const std::map<std::string,std::string>& criteria); 

    public:
        // Scheduler Service APIs
        /// Slamwarecore related APIs
        std::vector<detail::objects::ScheduledTask> getScheduledTasks();

        bool addScheduledTask(const detail::objects::ScheduledTask& task);

        detail::objects::ScheduledTask getScheduledTask(int taskId);

        detail::objects::ScheduledTask updateScheduledTask(const detail::objects::ScheduledTask& task);

        bool deleteScheduledTask(int taskId);

    public:
        // Recover localization by manual service APIs
        void recoverLocalizationByManual(const core::RectangleF& area, const features::motion_planner::RecoverLocalizationOptions& options = features::motion_planner::RecoverLocalizationOptions());
        rpos::core::ActionStatus getManualReLocalizationStatus();
        void cancelManualReLocalization();

    public:
        // Firmware Service APIs
        /// Base related APIs
        detail::objects::UpdateInfo getUpdateInfo();

        bool startFirmwareUpdate();

        detail::objects::UpdateProgress getFirmwareUpdateProgress();

    private:
        boost::shared_ptr<detail::SlamwareHttpsClient> getHttpsClient();

    private:
        boost::shared_ptr<impl_t> impl_;
    };
}}
