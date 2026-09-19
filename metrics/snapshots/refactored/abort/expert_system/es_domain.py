"""Final scenario-capable UAV Expert System knowledge base."""

from __future__ import annotations

from typing import Any, Callable

from mission_formalism_evaluation.expert_system.rule_engine import (
    FactMap,
    Rule,
)


PROPULSION_PRIORITY = 1
BATTERY_PRIORITY = 2
ABORT_PRIORITY = 3

GPS_FAILURE_PRIORITY = 4
GPS_RECOVERY_PRIORITY = 5
GPS_LOST_PRIORITY = 6

COMMUNICATION_FAILURE_PRIORITY = 7
COMMUNICATION_RECOVERY_PRIORITY = 8
COMMUNICATION_LOST_PRIORITY = 9

WIND_UNSAFE_PRIORITY = 10
SAFE_LAND_PRIORITY = 11
RETURN_HOME_PRIORITY = 12
SAFE_TERMINATION_PRIORITY = 13

NORMAL_PRIORITY = 20
COMPLETION_PRIORITY = 30

MISSION_TASK_TYPES = {
    "takeoff",
    "navigate",
    "land",
}


def mission_active(
    facts: FactMap,
) -> bool:
    """Return whether normal mission processing is active."""

    return (
        not facts.get("mission_complete", False)
        and not facts.get("all_tasks_processed", False)
        and not facts.get("mission_interrupted", False)
        and not facts.get("safe_terminated", False)
    )


def current_task_type(
    facts: FactMap,
) -> str:
    """Return normalized current task type."""

    return str(
        facts.get(
            "current_task_type",
            "",
        )
    ).lower()


def task_is(
    task_type: str,
) -> Callable[[FactMap], bool]:
    """Create condition for one mission task type."""

    def check(
        facts: FactMap,
    ) -> bool:

        return (
            mission_active(facts)
            and current_task_type(facts) == task_type
        )

    return check


def task_is_other(
    facts: FactMap,
) -> bool:
    """Match mission actions without a dedicated rule."""

    task_type = current_task_type(
        facts
    )

    return (
        mission_active(facts)
        and bool(task_type)
        and task_type not in MISSION_TASK_TYPES
    )


def mission_finished(
    facts: FactMap,
) -> bool:
    """Match normal mission completion."""

    return (
        facts.get("all_tasks_processed", False)
        and not facts.get("mission_complete", False)
        and not facts.get("safe_terminated", False)
    )


def propulsion_failed(
    facts: FactMap,
) -> bool:
    """Match propulsion failure."""

    return (
        facts.get("propulsion_failure", False)
        and facts.get("mission_interrupted", False)
        and not facts.get("safe_terminated", False)
    )


def battery_critical(
    facts: FactMap,
) -> bool:
    """Match critical battery."""

    return (
        facts.get("battery_critical", False)
        and facts.get("mission_interrupted", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("safe_terminated", False)
    )


def abort_requested(
    facts: FactMap,
) -> bool:
    """Match directed mission abort."""

    return (
        facts.get("abort_requested", False)
        and facts.get("mission_interrupted", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("battery_critical", False)
        and not facts.get("safe_terminated", False)
    )


def gps_lost(
    facts: FactMap,
) -> bool:
    """Match GPS loss."""

    return (
        facts.get("mission_interrupted", False)
        and facts.get("gps_was_lost", False)
        and not facts.get("gps_available", True)
        and not facts.get("gps_recovery_failed", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("battery_critical", False)
        and not facts.get("abort_requested", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def gps_restored(
    facts: FactMap,
) -> bool:
    """Match successful GPS recovery."""

    return (
        facts.get("mission_interrupted", False)
        and facts.get("gps_was_lost", False)
        and facts.get("gps_available", False)
        and not facts.get("gps_recovery_failed", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("battery_critical", False)
        and not facts.get("abort_requested", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def gps_failed(
    facts: FactMap,
) -> bool:
    """Match failed GPS recovery."""

    return (
        facts.get("mission_interrupted", False)
        and facts.get("gps_was_lost", False)
        and facts.get("gps_recovery_failed", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("battery_critical", False)
        and not facts.get("abort_requested", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def communication_lost(
    facts: FactMap,
) -> bool:
    """Match communication loss."""

    return (
        facts.get("mission_interrupted", False)
        and facts.get("communication_was_lost", False)
        and not facts.get("communication_available", True)
        and not facts.get(
            "communication_recovery_failed",
            False,
        )
        and not facts.get("propulsion_failure", False)
        and not facts.get("battery_critical", False)
        and not facts.get("abort_requested", False)
        and not facts.get("gps_was_lost", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def communication_restored(
    facts: FactMap,
) -> bool:
    """Match successful communication recovery."""

    return (
        facts.get("mission_interrupted", False)
        and facts.get("communication_was_lost", False)
        and facts.get("communication_available", False)
        and not facts.get(
            "communication_recovery_failed",
            False,
        )
        and not facts.get("propulsion_failure", False)
        and not facts.get("battery_critical", False)
        and not facts.get("abort_requested", False)
        and not facts.get("gps_was_lost", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def communication_failed(
    facts: FactMap,
) -> bool:
    """Match failed communication recovery."""

    return (
        facts.get("mission_interrupted", False)
        and facts.get("communication_was_lost", False)
        and facts.get(
            "communication_recovery_failed",
            False,
        )
        and not facts.get("propulsion_failure", False)
        and not facts.get("battery_critical", False)
        and not facts.get("abort_requested", False)
        and not facts.get("gps_was_lost", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def wind_unsafe(
    facts: FactMap,
) -> bool:
    """Match unsafe wind."""

    return (
        facts.get("wind_unsafe", False)
        and facts.get("mission_interrupted", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("battery_critical", False)
        and not facts.get("abort_requested", False)
        and not facts.get("safe_terminated", False)
    )


def safe_land_ready(
    facts: FactMap,
) -> bool:
    """Match safe-land transition."""

    return (
        facts.get("safe_land_ready", False)
        and not facts.get("safe_terminated", False)
    )


def return_home_ready(
    facts: FactMap,
) -> bool:
    """Match return-home transition."""

    return (
        facts.get("return_home_ready", False)
        and not facts.get("safe_terminated", False)
    )


def safe_termination_ready(
    facts: FactMap,
) -> bool:
    """Match safe termination."""

    return (
        facts.get("safe_termination_ready", False)
        and not facts.get("safe_terminated", False)
    )


def decision_action(
    decision: str,
) -> Callable[[FactMap], dict[str, Any]]:
    """Create action selecting one decision."""

    def action(
        _facts: FactMap,
    ) -> dict[str, Any]:

        return {
            "decision": decision,
        }

    return action


def complete_mission(
    _facts: FactMap,
) -> dict[str, Any]:
    """Mark normal mission completion."""

    return {
        "decision": "MISSION_COMPLETE",
        "mission_complete": True,
    }


def wait_for_gps(
    _facts: FactMap,
) -> dict[str, Any]:
    """Wait for GPS recovery."""

    return {
        "decision": "WAIT_FOR_GPS",
    }


def resume_after_gps(
    _facts: FactMap,
) -> dict[str, Any]:
    """Resume after GPS recovery."""

    return {
        "decision": "RESUME_MISSION",
        "mission_interrupted": False,
        "gps_was_lost": False,
        "gps_recovery_failed": False,
    }


def land_after_gps_failure(
    _facts: FactMap,
) -> dict[str, Any]:
    """Emergency land after GPS recovery failure."""

    return {
        "decision": "EMERGENCY_LAND",
        "mission_interrupted": False,
        "termination_reason": "GPS_RECOVERY_FAILED",
    }


def land_after_propulsion_failure(
    _facts: FactMap,
) -> dict[str, Any]:
    """Emergency land after propulsion failure."""

    return {
        "decision": "EMERGENCY_LAND",
        "mission_interrupted": False,
        "termination_reason": "PROPULSION_FAILURE",
    }


def land_after_battery_critical(
    _facts: FactMap,
) -> dict[str, Any]:
    """Emergency land after critical battery."""

    return {
        "decision": "EMERGENCY_LAND",
        "mission_interrupted": False,
        "termination_reason": "BATTERY_CRITICAL",
    }


def abort_after_request(
    _facts: FactMap,
) -> dict[str, Any]:
    """Start directed mission abort."""

    return {
        "decision": "ABORT_MISSION",
        "mission_interrupted": False,
        "termination_reason": "ABORT_REQUESTED",
    }


def wait_for_communication(
    _facts: FactMap,
) -> dict[str, Any]:
    """Wait for communication recovery."""

    return {
        "decision": "WAIT_FOR_COMMUNICATION",
    }


def resume_after_communication(
    _facts: FactMap,
) -> dict[str, Any]:
    """Resume after communication recovery."""

    return {
        "decision": "RESUME_MISSION",
        "mission_interrupted": False,
        "communication_available": True,
        "communication_was_lost": False,
        "communication_recovery_failed": False,
    }


def return_after_communication_failure(
    _facts: FactMap,
) -> dict[str, Any]:
    """Return home after communication recovery failure."""

    return {
        "decision": "RETURN_TO_HOME",
        "mission_interrupted": False,
        "termination_reason":
            "COMMUNICATION_RECOVERY_FAILED",
    }


def abort_after_unsafe_wind(
    _facts: FactMap,
) -> dict[str, Any]:
    """Abort after unsafe wind."""

    return {
        "decision": "ABORT_MISSION",
        "mission_interrupted": False,
        "termination_reason": "WIND_UNSAFE",
    }


def select_safe_land(
    _facts: FactMap,
) -> dict[str, Any]:
    """Select safe landing."""

    return {
        "decision": "SAFE_LAND",
        "safe_land_ready": False,
    }


def select_return_home(
    _facts: FactMap,
) -> dict[str, Any]:
    """Select return to home."""

    return {
        "decision": "RETURN_TO_HOME",
        "return_home_ready": False,
    }


def safe_terminate(
    _facts: FactMap,
) -> dict[str, Any]:
    """Safely terminate mission."""

    return {
        "decision": "SAFE_TERMINATE",
        "safe_terminated": True,
        "mission_complete": True,
        "safe_termination_ready": False,
    }


RUNTIME_EVENT_UPDATES: dict[
    str,
    dict[str, Any],
] = {
    "GPS_LOST": {
        "gps_available": False,
        "gps_was_lost": True,
        "gps_recovery_failed": False,
        "mission_interrupted": True,
    },
    "GPS_AVAILABLE": {
        "gps_available": True,
    },
    "GPS_RECOVERY_FAILED": {
        "gps_available": False,
        "gps_recovery_failed": True,
    },
    "PROPULSION_FAILURE": {
        "propulsion_failure": True,
        "mission_interrupted": True,
    },
    "BATTERY_CRITICAL": {
        "battery_critical": True,
        "mission_interrupted": True,
    },
    "ABORT_REQUESTED": {
        "abort_requested": True,
        "mission_interrupted": True,
    },
    "COMMUNICATION_LOST": {
        "communication_available": False,
        "communication_was_lost": True,
        "communication_recovery_failed": False,
        "mission_interrupted": True,
    },
    "COMMUNICATION_AVAILABLE": {
        "communication_available": True,
    },
    "COMMUNICATION_RECOVERY_FAILED": {
        "communication_available": False,
        "communication_recovery_failed": True,
    },
    "WIND_UNSAFE": {
        "wind_unsafe": True,
        "mission_interrupted": True,
    },
}


def build_knowledge_base() -> list[Rule]:
    """Build final scenario-capable knowledge base."""

    return [
        Rule(
            name="r_propulsion_failure",
            priority=PROPULSION_PRIORITY,
            condition=propulsion_failed,
            action=land_after_propulsion_failure,
        ),
        Rule(
            name="r_battery_critical",
            priority=BATTERY_PRIORITY,
            condition=battery_critical,
            action=land_after_battery_critical,
        ),
        Rule(
            name="r_abort_requested",
            priority=ABORT_PRIORITY,
            condition=abort_requested,
            action=abort_after_request,
        ),
        Rule(
            name="r_gps_recovery_failed",
            priority=GPS_FAILURE_PRIORITY,
            condition=gps_failed,
            action=land_after_gps_failure,
        ),
        Rule(
            name="r_gps_restored",
            priority=GPS_RECOVERY_PRIORITY,
            condition=gps_restored,
            action=resume_after_gps,
        ),
        Rule(
            name="r_gps_lost",
            priority=GPS_LOST_PRIORITY,
            condition=gps_lost,
            action=wait_for_gps,
        ),
        Rule(
            name="r_communication_recovery_failed",
            priority=COMMUNICATION_FAILURE_PRIORITY,
            condition=communication_failed,
            action=return_after_communication_failure,
        ),
        Rule(
            name="r_communication_restored",
            priority=COMMUNICATION_RECOVERY_PRIORITY,
            condition=communication_restored,
            action=resume_after_communication,
        ),
        Rule(
            name="r_communication_lost",
            priority=COMMUNICATION_LOST_PRIORITY,
            condition=communication_lost,
            action=wait_for_communication,
        ),
        Rule(
            name="r_wind_unsafe",
            priority=WIND_UNSAFE_PRIORITY,
            condition=wind_unsafe,
            action=abort_after_unsafe_wind,
        ),
        Rule(
            name="r_safe_land",
            priority=SAFE_LAND_PRIORITY,
            condition=safe_land_ready,
            action=select_safe_land,
        ),
        Rule(
            name="r_return_home",
            priority=RETURN_HOME_PRIORITY,
            condition=return_home_ready,
            action=select_return_home,
        ),
        Rule(
            name="r_safe_terminate",
            priority=SAFE_TERMINATION_PRIORITY,
            condition=safe_termination_ready,
            action=safe_terminate,
        ),
        Rule(
            name="r_takeoff",
            priority=NORMAL_PRIORITY,
            condition=task_is("takeoff"),
            action=decision_action("TAKEOFF"),
        ),
        Rule(
            name="r_navigate",
            priority=NORMAL_PRIORITY,
            condition=task_is("navigate"),
            action=decision_action("NAVIGATE"),
        ),
        Rule(
            name="r_land",
            priority=NORMAL_PRIORITY,
            condition=task_is("land"),
            action=decision_action("LAND"),
        ),
        Rule(
            name="r_mission_action",
            priority=NORMAL_PRIORITY,
            condition=task_is_other,
            action=decision_action("MISSION_ACTION"),
        ),
        Rule(
            name="r_mission_complete",
            priority=COMPLETION_PRIORITY,
            condition=mission_finished,
            action=complete_mission,
        ),
    ]
