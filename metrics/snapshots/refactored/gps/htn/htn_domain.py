"""GPS-capable UAV HTN domain."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


Task = tuple[str, dict[str, Any]]
WorldState = dict[str, Any]

Precondition = Callable[
    [WorldState, Task],
    bool,
]

Effect = Callable[
    [WorldState, Task],
    None,
]

Decomposition = Callable[
    [WorldState, Task],
    list[Task],
]


@dataclass(frozen=True)
class Operator:
    """Primitive executable HTN task."""

    name: str
    precondition: Precondition
    effect: Effect


@dataclass(frozen=True)
class Method:
    """Method that decomposes a compound HTN task."""

    name: str
    task_name: str
    precondition: Precondition
    decompose: Decomposition


class HTNDomain:
    """Collection of primitive operators and HTN methods."""

    def __init__(
        self,
    ) -> None:

        self.operators: dict[
            str,
            Operator,
        ] = {}

        self.methods: dict[
            str,
            list[Method],
        ] = {}

        self.compound_tasks: set[
            str
        ] = set()

    def add_operator(
        self,
        operator: Operator,
    ) -> None:
        """Register one primitive operator."""

        self.operators[
            operator.name
        ] = operator

    def add_method(
        self,
        method: Method,
    ) -> None:
        """Register one decomposition method."""

        self.compound_tasks.add(
            method.task_name
        )

        self.methods.setdefault(
            method.task_name,
            [],
        ).append(
            method
        )

    def is_primitive(
        self,
        task_name: str,
    ) -> bool:
        """Return whether task is primitive."""

        return (
            task_name
            in self.operators
        )

    def is_compound(
        self,
        task_name: str,
    ) -> bool:
        """Return whether task is compound."""

        return (
            task_name
            in self.compound_tasks
        )


def always(
    _state: WorldState,
    _task: Task,
) -> bool:
    """Always satisfy a precondition."""

    return True


def mission_available(
    state: WorldState,
    _task: Task,
) -> bool:
    """Allow execution while mission is active."""

    return not state[
        "safe_terminated"
    ]


def no_effect(
    _state: WorldState,
    _task: Task,
) -> None:
    """Apply no state change."""

    return


def can_takeoff(
    state: WorldState,
    _task: Task,
) -> bool:
    """Check takeoff precondition."""

    return (
        state["on_ground"]
        and not state["safe_terminated"]
    )


def takeoff_effect(
    state: WorldState,
    _task: Task,
) -> None:
    """Apply takeoff state change."""

    state[
        "on_ground"
    ] = False

    state[
        "airborne"
    ] = True


def can_navigate(
    state: WorldState,
    _task: Task,
) -> bool:
    """Check navigation precondition."""

    return (
        state["airborne"]
        and state["gps_available"]
        and not state["safe_terminated"]
    )


def navigate_effect(
    state: WorldState,
    task: Task,
) -> None:
    """Update current mission location."""

    payload = task[1]

    state[
        "location"
    ] = str(
        payload.get(
            "waypoint",
            payload.get(
                "name",
                "UNKNOWN",
            ),
        )
    )


def can_land(
    state: WorldState,
    _task: Task,
) -> bool:
    """Check landing precondition."""

    return (
        state["airborne"]
        and not state["safe_terminated"]
    )


def land_effect(
    state: WorldState,
    _task: Task,
) -> None:
    """Apply landing state change."""

    state[
        "airborne"
    ] = False

    state[
        "on_ground"
    ] = True


def resume_effect(
    state: WorldState,
    _task: Task,
) -> None:
    """Clear GPS interruption state."""

    state[
        "mission_interrupted"
    ] = False

    state[
        "gps_was_lost"
    ] = False

    state[
        "gps_recovery_failed"
    ] = False


def emergency_land_effect(
    state: WorldState,
    _task: Task,
) -> None:
    """Apply emergency landing."""

    state[
        "airborne"
    ] = False

    state[
        "on_ground"
    ] = True


def safe_terminate_effect(
    state: WorldState,
    _task: Task,
) -> None:
    """Mark mission safely terminated."""

    state[
        "safe_terminated"
    ] = True


def decompose_mission(
    _state: WorldState,
    task: Task,
) -> list[Task]:
    """Decompose mission into route execution."""

    return [
        (
            "execute_route",
            task[1],
        )
    ]


def decompose_route(
    _state: WorldState,
    task: Task,
) -> list[Task]:
    """Decompose route into mission tasks."""

    mission_tasks = (
        task[1][
            "mission_tasks"
        ]
    )

    return [
        (
            "perform_task",
            mission_task,
        )
        for mission_task
        in mission_tasks
    ]


def task_is(
    task_type: str,
) -> Precondition:
    """Build task-type precondition."""

    def check(
        _state: WorldState,
        task: Task,
    ) -> bool:

        return (
            str(
                task[1].get(
                    "type",
                    "",
                )
            ).lower()
            == task_type
        )

    return check


def task_is_other(
    _state: WorldState,
    task: Task,
) -> bool:
    """Match generic mission actions."""

    return (
        str(
            task[1].get(
                "type",
                "",
            )
        ).lower()
        not in {
            "takeoff",
            "navigate",
            "land",
        }
    )


def to_takeoff(
    _state: WorldState,
    task: Task,
) -> list[Task]:
    """Decompose mission task to takeoff."""

    return [
        (
            "takeoff",
            task[1],
        )
    ]


def to_navigate(
    _state: WorldState,
    task: Task,
) -> list[Task]:
    """Decompose mission task to navigation."""

    return [
        (
            "navigate",
            task[1],
        )
    ]


def to_land(
    _state: WorldState,
    task: Task,
) -> list[Task]:
    """Decompose mission task to landing."""

    return [
        (
            "land",
            task[1],
        )
    ]


def to_mission_action(
    _state: WorldState,
    task: Task,
) -> list[Task]:
    """Decompose generic mission action."""

    return [
        (
            "mission_action",
            task[1],
        )
    ]


def gps_lost(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match active GPS loss."""

    return (
        state["mission_interrupted"]
        and state["gps_was_lost"]
        and not state["gps_available"]
        and not state["gps_recovery_failed"]
    )


def gps_restored(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match successful GPS recovery."""

    return (
        state["mission_interrupted"]
        and state["gps_was_lost"]
        and state["gps_available"]
        and not state["gps_recovery_failed"]
    )


def gps_failed(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match failed GPS recovery."""

    return (
        state["mission_interrupted"]
        and state["gps_recovery_failed"]
    )


def wait_for_gps(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose GPS loss to waiting."""

    return [
        (
            "wait_for_gps",
            {},
        )
    ]


def resume_after_gps(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose GPS recovery to resume."""

    return [
        (
            "resume_mission",
            {},
        )
    ]


def land_after_gps_failure(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose failed recovery to safe landing."""

    return [
        (
            "emergency_land",
            {},
        ),
        (
            "safe_terminate",
            {
                "reason":
                    "GPS_RECOVERY_FAILED",
            },
        ),
    ]


RUNTIME_EVENT_UPDATES: dict[
    str,
    dict[str, Any],
] = {
    "GPS_LOST": {
        "gps_available":
            False,

        "gps_was_lost":
            True,

        "gps_recovery_failed":
            False,

        "mission_interrupted":
            True,
    },

    "GPS_AVAILABLE": {
        "gps_available":
            True,
    },

    "GPS_RECOVERY_FAILED": {
        "gps_available":
            False,

        "gps_recovery_failed":
            True,
    },
}


def build_uav_domain(
) -> HTNDomain:
    """Build GPS-capable UAV HTN domain."""

    domain = HTNDomain()

    operators = [
        Operator(
            "takeoff",
            can_takeoff,
            takeoff_effect,
        ),

        Operator(
            "navigate",
            can_navigate,
            navigate_effect,
        ),

        Operator(
            "mission_action",
            mission_available,
            no_effect,
        ),

        Operator(
            "land",
            can_land,
            land_effect,
        ),

        Operator(
            "wait_for_gps",
            always,
            no_effect,
        ),

        Operator(
            "resume_mission",
            always,
            resume_effect,
        ),

        Operator(
            "emergency_land",
            can_land,
            emergency_land_effect,
        ),

        Operator(
            "safe_terminate",
            always,
            safe_terminate_effect,
        ),
    ]

    methods = [
        Method(
            "m_execute_mission",
            "execute_mission",
            mission_available,
            decompose_mission,
        ),

        Method(
            "m_execute_route",
            "execute_route",
            mission_available,
            decompose_route,
        ),

        Method(
            "m_takeoff",
            "perform_task",
            task_is(
                "takeoff"
            ),
            to_takeoff,
        ),

        Method(
            "m_navigate",
            "perform_task",
            task_is(
                "navigate"
            ),
            to_navigate,
        ),

        Method(
            "m_land",
            "perform_task",
            task_is(
                "land"
            ),
            to_land,
        ),

        Method(
            "m_other_action",
            "perform_task",
            task_is_other,
            to_mission_action,
        ),

        Method(
            "m_gps_recovery_failed",
            "handle_runtime_condition",
            gps_failed,
            land_after_gps_failure,
        ),

        Method(
            "m_gps_restored",
            "handle_runtime_condition",
            gps_restored,
            resume_after_gps,
        ),

        Method(
            "m_gps_lost",
            "handle_runtime_condition",
            gps_lost,
            wait_for_gps,
        ),
    ]

    for operator in operators:

        domain.add_operator(
            operator
        )

    for method in methods:

        domain.add_method(
            method
        )

    return domain
