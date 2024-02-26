/*
* power_management.h
* Power management features
*
* Created by Tony Huang (tony@slamtec.com) at 2017-2-25
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace features { namespace system_resource {

    /**
    * @brief Sleep mode of device
    */
    enum SleepMode {
        /**
        * @brief The targeting core firmware doens't support this command
        */
        SleepModeUnknown,

        /**
        * @brief The device is awake, it will response to commands immediately
        */
        SleepModeAwake,

        /**
        * @brief The device is waking up, please wait for some time
        */
        SleepModeWakingUp,

        /**
        * @brief The device is asleep, it might take some time to wake device up
        */
        SleepModeAsleep,

        /**
         * @brief The device is going to sleep
        */
        SleepModeGoingToSleep,
    };

    /**
    * @brief Docking status of the robot
    */
    enum DockingStatus {
        /**
        * @brief The docking status is unknown (doens't supported by the targeting Slamware firmware)
        */
        DockingStatusUnknown,

        /**
        * @brief The robot is on the dock
        */
        DockingStatusOnDock,

        /**
        * @brief The robot is not on the dock
        */
        DockingStatusNotOnDock
    };

    /**
    * @brief Power stage of robot
    */
    enum PowerStage {
        /**
        * @brief The robot power stage unknown.
        */
        PowerStageUnknown,
        /**
        * @brief The robot is starting.
        */
        PowerStageStarting,
        /**
        * @brief The robot is running.
        */
        PowerStageRunning,
        /**
        * @brief The robot is restarting.
        */
        PowerStageRestarting,
        /**
        * @brief The robot is shuting down.
        */
        PowerStageShutingDown,
        /**
        * @brief The robot power stage error.
        */
        PowerStageError
    };

    /**
    * @brief Composed power status of the device
    */
    struct RPOS_CORE_API PowerStatus {
        /**
        * @brief The dc cord is connected
        */
        bool isDCConnected;

        /**
        * @brief The robot is on dock
        */
        DockingStatus dockingStatus;

        /**
        * @brief The robot is charging
        */
        bool isCharging;

        /**
        * @brief Battery level (0 ~ 100)
        */
        int batteryPercentage;

        /**
        * @brief Robot sleep mode
        */
        SleepMode sleepMode;

        /**
        * @brief Robot power stage
        */
        PowerStage powerStage;

        PowerStatus();
        ~PowerStatus();
    };

} } }
