"""
GPS, propulsion, communication, and wind-capable UAV Expert System Knowledge Base.
"""

from __future__ import annotations

from typing import Any

from mission_formalism_evaluation.expert_system.rule_engine import (
    FactMap,
    Rule,
)


PROPULSION_PRIORITY = 1

GPS_FAILURE_PRIORITY = 2
GPS_RECOVERY_PRIORITY = 3
GPS_LOST_PRIORITY = 4

COMMUNICATION_FAILURE_PRIORITY = 5
COMMUNICATION_RECOVERY_PRIORITY = 6
COMMUNICATION_LOST_PRIORITY = 7

WIND_UNSAFE_PRIORITY = 8
SAFE_LAND_PRIORITY = 9
SAFE_TERMINATION_PRIORITY = 10

NORMAL_PRIORITY = 20
COMPLETION_PRIORITY = 30


# ================================================================
# Nominal mission conditions
# ================================================================

def task_is(task_type: str):

    def check(facts: FactMap) -> bool:

        return (
            not facts.get("mission_complete", False)
            and not facts.get("all_tasks_processed", False)
            and not facts.get("mission_interrupted", False)
            and str(
                facts.get(
                    "current_task_type",
                    "",
                )
            ).lower()
            == task_type
        )

    return check


def task_is_other(facts: FactMap) -> bool:

    task_type = str(
        facts.get(
            "current_task_type",
            "",
        )
    ).lower()

    return (
        not facts.get("mission_complete", False)
        and not facts.get("all_tasks_processed", False)
        and not facts.get("mission_interrupted", False)
        and bool(task_type)
        and task_type
        not in {
            "takeoff",
            "navigate",
            "land",
        }
    )


def mission_finished(facts: FactMap) -> bool:

    return (
        facts.get("all_tasks_processed", False)
        and not facts.get("mission_complete", False)
        and not facts.get("safe_terminated", False)
    )


# ================================================================
# GPS conditions
# ================================================================

def gps_lost(facts: FactMap) -> bool:

    return (
        facts.get("mission_interrupted", False)
        and facts.get("gps_was_lost", False)
        and not facts.get("gps_available", True)
        and not facts.get("gps_recovery_failed", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def gps_restored(facts: FactMap) -> bool:

    return (
        facts.get("mission_interrupted", False)
        and facts.get("gps_was_lost", False)
        and facts.get("gps_available", False)
        and not facts.get("gps_recovery_failed", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def gps_failed(facts: FactMap) -> bool:

    return (
        facts.get("mission_interrupted", False)
        and facts.get("gps_was_lost", False)
        and facts.get("gps_recovery_failed", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


# ================================================================
# Propulsion condition
# ================================================================

def propulsion_failed(facts: FactMap) -> bool:

    return (
        facts.get("propulsion_failure", False)
        and facts.get("mission_interrupted", False)
        and not facts.get("safe_terminated", False)
    )


# ================================================================
# Communication conditions
# ================================================================

def communication_lost(facts: FactMap) -> bool:

    return (
        facts.get("mission_interrupted", False)
        and facts.get("communication_was_lost", False)
        and not facts.get("communication_available", True)
        and not facts.get(
            "communication_recovery_failed",
            False,
        )
        and not facts.get("propulsion_failure", False)
        and not facts.get("gps_was_lost", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def communication_restored(facts: FactMap) -> bool:

    return (
        facts.get("mission_interrupted", False)
        and facts.get("communication_was_lost", False)
        and facts.get("communication_available", False)
        and not facts.get(
            "communication_recovery_failed",
            False,
        )
        and not facts.get("propulsion_failure", False)
        and not facts.get("gps_was_lost", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


def communication_failed(facts: FactMap) -> bool:

    return (
        facts.get("mission_interrupted", False)
        and facts.get("communication_was_lost", False)
        and facts.get(
            "communication_recovery_failed",
            False,
        )
        and not facts.get("propulsion_failure", False)
        and not facts.get("gps_was_lost", False)
        and not facts.get("wind_unsafe", False)
        and not facts.get("safe_terminated", False)
    )


# ================================================================
# Wind conditions
# ================================================================

def wind_unsafe(facts: FactMap) -> bool:

    return (
        facts.get("wind_unsafe", False)
        and facts.get("mission_interrupted", False)
        and not facts.get("propulsion_failure", False)
        and not facts.get("safe_terminated", False)
    )


def safe_land_ready(facts: FactMap) -> bool:

    return (
        facts.get("safe_land_ready", False)
        and not facts.get("safe_terminated", False)
    )


def safe_termination_ready(facts: FactMap) -> bool:

    return (
        facts.get("safe_termination_ready", False)
        and not facts.get("safe_terminated", False)
    )


# ================================================================
# Nominal mission actions
# ================================================================

def select_takeoff(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "TAKEOFF",
    }


def select_navigate(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "NAVIGATE",
    }


def select_mission_action(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "MISSION_ACTION",
    }


def select_land(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "LAND",
    }


def complete_mission(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "MISSION_COMPLETE",
        "mission_complete": True,
    }


# ================================================================
# Recovery actions
# ================================================================

def wait_for_gps(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "WAIT_FOR_GPS",
    }


def resume_after_gps(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "RESUME_MISSION",
        "mission_interrupted": False,
        "gps_was_lost": False,
        "gps_recovery_failed": False,
    }


def land_after_gps_failure(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "EMERGENCY_LAND",
        "mission_interrupted": False,
        "termination_reason":
            "GPS_RECOVERY_FAILED",
    }


def land_after_propulsion_failure(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "EMERGENCY_LAND",
        "mission_interrupted": False,
        "termination_reason":
            "PROPULSION_FAILURE",
    }


def wait_for_communication(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "WAIT_FOR_COMMUNICATION",
    }


def resume_after_communication(
    _facts: FactMap,
) -> dict[str, Any]:

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

    return {
        "decision": "RETURN_TO_HOME",
        "mission_interrupted": False,
        "termination_reason":
            "COMMUNICATION_RECOVERY_FAILED",
    }


def abort_after_unsafe_wind(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "ABORT_MISSION",
        "mission_interrupted": False,
        "termination_reason": "WIND_UNSAFE",
    }


def select_safe_land(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "SAFE_LAND",
        "safe_land_ready": False,
    }


def safe_terminate(
    _facts: FactMap,
) -> dict[str, Any]:

    return {
        "decision": "SAFE_TERMINATE",
        "safe_terminated": True,
        "mission_complete": True,
        "safe_termination_ready": False,
    }


# ================================================================
# Event-to-fact translation
# ================================================================

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

    "COMMUNICATION_LOST": {
        "communication_available": False,
        "communication_was_lost": True,
        "communication_recovery_failed": False,
        "mission_interrupted": True,
    },

    "COMMUNICATION_RESTORED": {
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


# ================================================================
# Knowledge Base construction
# ================================================================

def build_knowledge_base() -> list[Rule]:
    """Build the scenario-capable Knowledge Base."""

    return [
        Rule(
            name="r_propulsion_failure",
            priority=PROPULSION_PRIORITY,
            condition=propulsion_failed,
            action=land_after_propulsion_failure,
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
            name="r_safe_terminate",
            priority=SAFE_TERMINATION_PRIORITY,
            condition=safe_termination_ready,
            action=safe_terminate,
        ),

        Rule(
            name="r_takeoff",
            priority=NORMAL_PRIORITY,
            condition=task_is("takeoff"),
            action=select_takeoff,
        ),

        Rule(
            name="r_navigate",
            priority=NORMAL_PRIORITY,
            condition=task_is("navigate"),
            action=select_navigate,
        ),

        Rule(
            name="r_land",
            priority=NORMAL_PRIORITY,
            condition=task_is("land"),
            action=select_land,
        ),

        Rule(
            name="r_mission_action",
            priority=NORMAL_PRIORITY,
            condition=task_is_other,
            action=select_mission_action,
        ),

        Rule(
            name="r_mission_complete",
            priority=COMPLETION_PRIORITY,
            condition=mission_finished,
            action=complete_mission,
        ),
    ]
