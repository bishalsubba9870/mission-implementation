"""Nominal UAV Expert System knowledge base."""

from __future__ import annotations

from typing import Any, Callable

from mission_formalism_evaluation.expert_system.rule_engine import (
    FactMap,
    Rule,
)


NORMAL_PRIORITY = 10
COMPLETION_PRIORITY = 20

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
    """Create a condition for one mission task type."""

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
    """Match mission actions without a dedicated production rule."""

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
    """Match completion after all mission tasks are processed."""

    return (
        facts.get("all_tasks_processed", False)
        and not facts.get("mission_complete", False)
    )


def decision_action(
    decision: str,
) -> Callable[[FactMap], dict[str, Any]]:
    """Create an action that selects one executor decision."""

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
    """Mark the mission complete."""

    return {
        "decision": "MISSION_COMPLETE",
        "mission_complete": True,
    }


def build_nominal_knowledge_base() -> list[Rule]:
    """Build the nominal production-rule knowledge base."""

    return [
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
