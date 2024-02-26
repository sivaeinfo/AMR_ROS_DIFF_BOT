#pragma once

#include <rpos/core/rpos_core_config.h>

#include <cstdint>

namespace rpos { namespace core {

    struct RPOS_CORE_API SlamcoreShutdownParam
    {
        std::uint32_t restartTimeIntervalMinute;
        std::uint32_t shutdownTimeIntervalMinute;
        bool resetShutdownRestartTimeInterval;

        SlamcoreShutdownParam();
    };

    struct RPOS_CORE_API SetChargeControl
    {
        std::uint32_t chargeControl;    //=1: disable charging; =0: enable charging

        SetChargeControl();
    };

    struct RPOS_CORE_API GetChargeControlStatus
    {
        std::uint32_t chargeControlStatus;  //=1: disabled charging; =0: enabled charging

        GetChargeControlStatus();
    };

    struct RPOS_CORE_API ImuTreshTestData
    {
        float check_cov_roll;
        float check_cov_pitch;
        float check_cov_yaw;
        float gyro_cov_roll;
        float gyro_cov_pitch;
        float gyro_cov_yaw;
        std::uint32_t thresh;
        float sum_yaw;
        std::uint32_t time_stamp;

        ImuTreshTestData();
    };

    struct IMURawADCData
    {
        int acc_x;
        int acc_y;
        int acc_z;
        int gyro_x;
        int gyro_y;
        int gyro_z;
        int comp_x;
        int comp_y;
        int comp_z;

        uint32_t timestamp;
        IMURawADCData();
    };

}}
