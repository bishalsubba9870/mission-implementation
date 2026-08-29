"""
Nominal UAV HTN domain.

This file contains only normal mission execution.

There is intentionally no runtime fault handling here.
GPS loss, communication loss, propulsion failure,
unsafe wind, critical battery, and operator abort
will be introduced incrementally in later snapshots.

Algorithmic basis:
    Nau et al., SHOP2, JAIR 2003.

Modeling terminology:
    Höller et al., HDDL, AAAI 2020.
"""

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

        self.operators[
            operator.name
        ] = operator

    def add_method(
        self,
        method: Method,
    ) -> None:

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

        return (
            task_name
            in self.operators
        )

    def is_compound(
        self,
        task_name: str,
    ) -> bool:

        return (
            task_name
            in self.compound_tasks
        )


# ================================================================
# Generic predicates
# ================================================================

def always(
    _state: WorldState,
    _task: Task,
) -> bool:

    return True


def no_effect(
    _state: WorldState,
    _task: Task,
) -> None:

    return


# ================================================================
# Primitive operators
# ================================================================

def can_takeoff(
    state: WorldState,
    _task: Task,
) -> bool:

    return state[
        "on_ground"
    ]


def takeoff_effect(
    state: WorldState,
    _task: Task,
) -> None:

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

    return state[
        "airborne"
    ]


def navigate_effect(
    state: WorldState,
    task: Task,
) -> None:

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

    return state[
        "airborne"
    ]


def land_effect(
    state: WorldState,
    _task: Task,
) -> None:

    state[
        "airborne"
    ] = False

    state[
        "on_ground"
    ] = True


# ================================================================
# Mission hierarchy
# ================================================================

def mission_available(
    _state: WorldState,
    _task: Task,
) -> bool:

    return True


def decompose_mission(
    _state: WorldState,
    task: Task,
) -> list[Task]:

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


# ================================================================
# Mission-task decomposition
# ================================================================

def task_is(
    task_type: str,
) -> Precondition:

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

    return [
        (
            "mission_action",
            task[1],
        )
    ]


# ================================================================
# Domain construction
# ================================================================

def build_uav_domain(
) -> HTNDomain:
    """Build the nominal UAV HTN domain."""

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
            always,
            no_effect,
        ),

        Operator(
            "land",
            can_land,
            land_effect,
        ),
    ]

    for operator in operators:
        domain.add_operator(
            operator
        )

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
    ]

    for method in methods:
        domain.add_method(
            method
        )

    return domain
