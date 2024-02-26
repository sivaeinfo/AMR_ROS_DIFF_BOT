/**
* feature.h
* The Sweep Motion Planner feature
*
* Created By Tony Huang @ 2015-4-3
* Copyright (c) 2015 Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/features/motion_planner/feature.h>
#include <rpos/features/sweep_motion_planner/sweep_move_action.h>
#include <rpos/features/sweep_motion_planner/sweep_region.h>

namespace rpos { namespace features { 

    namespace detail {
        class SweepMotionPlannerImpl;
    } 

    struct RPOS_CORE_API SweepMotionOptions
    {
        SweepMotionOptions();
        std::vector<rpos::core::SegmentID> region_ids;
        boost::optional<float> spacing;
    };

    class RPOS_CORE_API SweepMotionPlanner : public rpos::core::Feature {
    public:
        typedef detail::SweepMotionPlannerImpl impl_t;

        RPOS_OBJECT_CTORS_WITH_BASE(SweepMotionPlanner, rpos::core::Feature);
        SweepMotionPlanner(boost::shared_ptr<impl_t> impl);
        ~SweepMotionPlanner();

    public:
        rpos::actions::SweepMoveAction startSweep();
        rpos::actions::SweepMoveAction startRegionSweep(const SweepMotionOptions& options);
        float getSweepArea();
    };

} } 

namespace rpos { namespace core { namespace detail {

    template <>
    struct RPOS_CORE_API feature_caster < features::SweepMotionPlanner >
    {
        static features::SweepMotionPlanner cast(Feature& v);
    };

} } }
