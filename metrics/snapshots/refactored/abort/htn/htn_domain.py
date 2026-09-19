"""Fully scenario-capable UAV HTN domain."""

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
        and state["communication_available"]
        and not state["propulsion_failure"]
        and not state["wind_unsafe"]
        and not state["battery_critical"]
        and not state["abort_requested"]
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


def safe_land_effect(
    state: WorldState,
    _task: Task,
) -> None:
    """Apply safe landing."""

    state[
        "airborne"
    ] = False

    state[
        "on_ground"
    ] = True


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


def rtl_effect(
    state: WorldState,
    _task: Task,
) -> None:
    """Apply return-to-home action."""

    state[
        "location"
    ] = "HOME"


def abort_mission_effect(
    state: WorldState,
    _task: Task,
) -> None:
    """Apply mission abort."""

    state[
        "mission_aborted"
    ] = True

    state[
        "mission_interrupted"
    ] = True


def resume_effect(
    state: WorldState,
    _task: Task,
) -> None:
    """Clear recoverable interruption state."""

    state[
        "mission_interrupted"
    ] = False

    state[
        "gps_was_lost"
    ] = False

    state[
        "gps_recovery_failed"
    ] = False

    state[
        "communication_was_lost"
    ] = False

    state[
        "communication_recovery_failed"
    ] = False


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
        and not state["propulsion_failure"]
        and not state["wind_unsafe"]
        and not state["battery_critical"]
        and not state["abort_requested"]
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
        and not state["propulsion_failure"]
        and not state["wind_unsafe"]
        and not state["battery_critical"]
        and not state["abort_requested"]
    )


def gps_failed(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match failed GPS recovery."""

    return (
        state["mission_interrupted"]
        and state["gps_recovery_failed"]
        and not state["propulsion_failure"]
        and not state["wind_unsafe"]
        and not state["battery_critical"]
        and not state["abort_requested"]
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
    """Decompose failed GPS recovery."""

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


def propulsion_failed(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match active propulsion failure."""

    return (
        state["propulsion_failure"]
        and not state["safe_terminated"]
    )


def land_after_propulsion_failure(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose propulsion failure."""

    return [
        (
            "emergency_land",
            {},
        ),
        (
            "safe_terminate",
            {
                "reason":
                    "PROPULSION_FAILURE",
            },
        ),
    ]


def communication_lost(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match active communication loss."""

    return (
        state["mission_interrupted"]
        and state["communication_was_lost"]
        and not state["communication_available"]
        and not state["communication_recovery_failed"]
        and not state["propulsion_failure"]
        and not state["wind_unsafe"]
        and not state["battery_critical"]
        and not state["abort_requested"]
    )


def communication_restored(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match successful communication recovery."""

    return (
        state["mission_interrupted"]
        and state["communication_was_lost"]
        and state["communication_available"]
        and not state["communication_recovery_failed"]
        and not state["propulsion_failure"]
        and not state["wind_unsafe"]
        and not state["battery_critical"]
        and not state["abort_requested"]
    )


def communication_failed(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match failed communication recovery."""

    return (
        state["mission_interrupted"]
        and state["communication_recovery_failed"]
        and not state["propulsion_failure"]
        and not state["wind_unsafe"]
        and not state["battery_critical"]
        and not state["abort_requested"]
    )


def wait_for_communication(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose communication loss to waiting."""

    return [
        (
            "wait_for_communication",
            {},
        )
    ]


def resume_after_communication(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose communication recovery to resume."""

    return [
        (
            "resume_mission",
            {},
        )
    ]


def rtl_after_communication_failure(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose failed communication recovery."""

    return [
        (
            "return_to_home",
            {},
        ),
        (
            "safe_terminate",
            {
                "reason":
                    "COMMUNICATION_RECOVERY_FAILED",
            },
        ),
    ]


def wind_unsafe(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match unsafe wind."""

    return (
        state["wind_unsafe"]
        and not state["safe_terminated"]
    )


def terminate_after_unsafe_wind(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose unsafe-wind response."""

    return [
        (
            "abort_mission",
            {},
        ),
        (
            "safe_land",
            {},
        ),
        (
            "safe_terminate",
            {
                "reason":
                    "WIND_UNSAFE",
            },
        ),
    ]


def battery_critical(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match critical battery."""

    return (
        state["battery_critical"]
        and not state["safe_terminated"]
    )


def land_after_battery_critical(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose critical-battery response."""

    return [
        (
            "emergency_land",
            {},
        ),
        (
            "safe_terminate",
            {
                "reason":
                    "BATTERY_CRITICAL",
            },
        ),
    ]


def abort_requested(
    state: WorldState,
    _task: Task,
) -> bool:
    """Match external abort request."""

    return (
        state["abort_requested"]
        and not state["safe_terminated"]
    )


def rtl_after_abort_request(
    _state: WorldState,
    _task: Task,
) -> list[Task]:
    """Decompose external abort request."""

    return [
        (
            "return_to_home",
            {},
        ),
        (
            "safe_terminate",
            {
                "reason":
                    "ABORT_REQUESTED",
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

    "PROPULSION_FAILURE": {
        "propulsion_failure":
            True,

        "mission_interrupted":
            True,
    },

    "COMMUNICATION_LOST": {
        "communication_available":
            False,

        "communication_was_lost":
            True,

        "communication_recovery_failed":
            False,

        "mission_interrupted":
            True,
    },

    "COMMUNICATION_AVAILABLE": {
        "communication_available":
            True,
    },

    "COMMUNICATION_RECOVERY_FAILED": {
        "communication_available":
            False,

        "communication_recovery_failed":
            True,
    },

    "WIND_UNSAFE": {
        "wind_unsafe":
            True,

        "mission_interrupted":
            True,
    },

    "BATTERY_CRITICAL": {
        "battery_critical":
            True,

        "mission_interrupted":
            True,
    },

    "ABORT_REQUESTED": {
        "abort_requested":
            True,

        "mission_interrupted":
            True,
    },
}


def build_uav_domain(
) -> HTNDomain:
    """Build fully scenario-capable UAV HTN domain."""

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
            "wait_for_communication",
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
            "return_to_home",
            always,
            rtl_effect,
        ),

        Operator(
            "abort_mission",
            always,
            abort_mission_effect,
        ),

        Operator(
            "safe_land",
            can_land,
            safe_land_effect,
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
            "m_propulsion_failure",
            "handle_runtime_condition",
            propulsion_failed,
            land_after_propulsion_failure,
        ),

        Method(
            "m_wind_unsafe",
            "handle_runtime_condition",
            wind_unsafe,
            terminate_after_unsafe_wind,
        ),

        Method(
            "m_battery_critical",
            "handle_runtime_condition",
            battery_critical,
            land_after_battery_critical,
        ),

        Method(
            "m_abort_requested",
            "handle_runtime_condition",
            abort_requested,
            rtl_after_abort_request,
        ),

        Method(
            "m_communication_recovery_failed",
            "handle_runtime_condition",
            communication_failed,
            rtl_after_communication_failure,
        ),

        Method(
            "m_communication_restored",
            "handle_runtime_condition",
            communication_restored,
            resume_after_communication,
        ),

        Method(
            "m_communication_lost",
            "handle_runtime_condition",
            communication_lost,
            wait_for_communication,
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
