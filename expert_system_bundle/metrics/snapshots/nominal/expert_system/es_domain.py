"""
Nominal UAV Expert System Knowledge Base.
"""

from __future__ import annotations

from typing import Any

from mission_formalism_evaluation.expert_system.rule_engine import (
    FactMap,
    Rule,
)


NORMAL_PRIORITY = 10
COMPLETION_PRIORITY = 20


def task_is(
    task_type: str,
):
    def check(
        facts: FactMap,
    ) -> bool:

        return (
            not facts.get(
                "mission_complete",
                False,
            )
            and not facts.get(
                "all_tasks_processed",
                False,
            )
            and str(
                facts.get(
                    "current_task_type",
                    "",
                )
            ).lower()
            == task_type
        )

    return check


def task_is_other(
    facts: FactMap,
) -> bool:

    task_type = str(
        facts.get(
            "current_task_type",
            "",
        )
    ).lower()

    return (
        not facts.get(
            "mission_complete",
            False,
        )
        and not facts.get(
            "all_tasks_processed",
            False,
        )
        and bool(
            task_type
        )
        and task_type
        not in {
            "takeoff",
            "navigate",
            "land",
        }
    )


def mission_finished(
    facts: FactMap,
) -> bool:

    return (
        facts.get(
            "all_tasks_processed",
            False,
        )
        and not facts.get(
            "mission_complete",
            False,
        )
    )


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


def build_nominal_knowledge_base(
) -> list[Rule]:
    """Build the nominal UAV production-rule Knowledge Base."""

    return [
        Rule(
            name="r_takeoff",
            priority=NORMAL_PRIORITY,
            condition=task_is(
                "takeoff"
            ),
            action=select_takeoff,
        ),

        Rule(
            name="r_navigate",
            priority=NORMAL_PRIORITY,
            condition=task_is(
                "navigate"
            ),
            action=select_navigate,
        ),

        Rule(
            name="r_land",
            priority=NORMAL_PRIORITY,
            condition=task_is(
                "land"
            ),
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
