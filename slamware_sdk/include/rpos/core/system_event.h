#pragma once

namespace rpos {
    namespace core {
        // @attention: it must be added at the end.
        enum InternalSystemEvent
        {
            InternalSystemEventRelocalizationFail = 1,
            InternalSystemEventRelocalizationSuccess,
            InternalSystemEventBackHomeFail,
            InternalSystemEventAlignMap,
            InternalSystemEventSideBrushStall,
            InternalSystemEventRollingBrushStall,
            InternalSystemEventBumperDown,
            InternalSystemEventWallIrDown,
            InternalSystemEventCliffDown,
            InternalSystemEventBlowerStall,
            InternalSystemEventDustbinGone,
            InternalSystemEventMotionJam,
            InternalSystemEventIRDown,
            InternalSystemEventIMUDown,
            InternalSystemEventMopDown,
            InternalSystemEventMopUp,
            InternalSystemEventSetMapDone,
            InternalSystemEventSetPoseDone,
            InternalSystemEventMapUpdateEnable,
            InternalSystemEventMapUpdateDisable,
            InternalSystemEventPathOccupied,
            InternalSystemEventTrappedInForbiddenArea,
            InternalSystemEventIsInElevatorArea,
            InternalSystemEventIsOutOfElevatorArea,
            InternalSystemEventMapLoopClosure,
            InternalSystemEventEnterElevatorFailed,
            InternalSystemEventSearchElevatorPathFailed,
            InternalSystemEventCliffTriggered,
            InternalSystemEventSearchLocalPathFailed,
            InternalSystemEventWaitPlanningFailed,
            InternalSystemEventWaitPlanningFoundNotGoodPath,
            InternalSystemEventPowerOff,
            InternalSystemEventPathFinderFailed,
            InternalSystemEventTakeElevatorOccupied,
            InternalSystemEventTurningRoundInElevator,
            InternalSystemEventEnterElevatorPathFound,
            InternalSystemEventNarrowCorridor
        };

    }
}