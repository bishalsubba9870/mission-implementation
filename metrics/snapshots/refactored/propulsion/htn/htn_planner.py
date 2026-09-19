"""Generic totally ordered state-based HTN planner."""

from __future__ import annotations

import copy

from dataclasses import dataclass
from dataclasses import field

from mission_formalism_evaluation.htn.htn_domain import (
    HTNDomain,
    Task,
    WorldState,
)


@dataclass
class PlanningTrace:
    """Record structures used during planning."""

    primitive_tasks: list[
        str
    ] = field(
        default_factory=list
    )

    compound_tasks: list[
        str
    ] = field(
        default_factory=list
    )

    methods: list[
        str
    ] = field(
        default_factory=list
    )


@dataclass
class PlanningResult:
    """Store one HTN planning result."""

    success: bool

    plan: list[
        Task
    ]

    final_state: WorldState

    trace: PlanningTrace

    reason: str = ""


class HTNPlanner:
    """Perform totally ordered HTN decomposition."""

    def __init__(
        self,
        domain: HTNDomain,
    ) -> None:

        self.domain = domain

    def plan(
        self,
        state: WorldState,
        tasks: list[Task],
    ) -> PlanningResult:
        """Generate a primitive task plan."""

        result = self._seek(
            state=copy.deepcopy(
                state
            ),
            tasks=copy.deepcopy(
                tasks
            ),
            plan=[],
            trace=PlanningTrace(),
        )

        if result is None:

            return self._failure_result(
                state
            )

        (
            plan,
            final_state,
            trace,
        ) = result

        return PlanningResult(
            success=True,
            plan=plan,
            final_state=final_state,
            trace=trace,
        )

    def _seek(
        self,
        state: WorldState,
        tasks: list[Task],
        plan: list[Task],
        trace: PlanningTrace,
    ) -> (
        tuple[
            list[Task],
            WorldState,
            PlanningTrace,
        ]
        | None
    ):
        """Recursively search for a valid decomposition."""

        if not tasks:

            return (
                plan,
                state,
                trace,
            )

        task = tasks[0]
        remaining = tasks[1:]

        task_name = task[0]

        if self.domain.is_primitive(
            task_name
        ):

            return self._seek_primitive(
                state,
                task,
                remaining,
                plan,
                trace,
            )

        if self.domain.is_compound(
            task_name
        ):

            return self._seek_compound(
                state,
                task,
                remaining,
                plan,
                trace,
            )

        raise ValueError(
            "HTN task is neither "
            "primitive nor compound: "
            f"{task_name}"
        )

    def _seek_primitive(
        self,
        state: WorldState,
        task: Task,
        remaining: list[Task],
        plan: list[Task],
        trace: PlanningTrace,
    ):
        """Apply one primitive operator."""

        task_name = task[0]

        operator = (
            self.domain
            .operators[
                task_name
            ]
        )

        if not operator.precondition(
            state,
            task,
        ):

            return None

        next_state = copy.deepcopy(
            state
        )

        operator.effect(
            next_state,
            task,
        )

        next_plan = copy.deepcopy(
            plan
        )

        next_plan.append(
            copy.deepcopy(
                task
            )
        )

        next_trace = copy.deepcopy(
            trace
        )

        next_trace.primitive_tasks.append(
            task_name
        )

        return self._seek(
            state=next_state,
            tasks=copy.deepcopy(
                remaining
            ),
            plan=next_plan,
            trace=next_trace,
        )

    def _seek_compound(
        self,
        state: WorldState,
        task: Task,
        remaining: list[Task],
        plan: list[Task],
        trace: PlanningTrace,
    ):
        """Try methods for one compound task."""

        task_name = task[0]

        methods = (
            self.domain
            .methods[
                task_name
            ]
        )

        for method in methods:

            result = self._try_method(
                method,
                state,
                task,
                remaining,
                plan,
                trace,
            )

            if result is not None:

                return result

        return None

    def _try_method(
        self,
        method,
        state: WorldState,
        task: Task,
        remaining: list[Task],
        plan: list[Task],
        trace: PlanningTrace,
    ):
        """Try one decomposition method."""

        method_state = copy.deepcopy(
            state
        )

        if not method.precondition(
            method_state,
            task,
        ):

            return None

        subtasks = method.decompose(
            method_state,
            task,
        )

        next_trace = copy.deepcopy(
            trace
        )

        next_trace.compound_tasks.append(
            task[0]
        )

        next_trace.methods.append(
            method.name
        )

        next_tasks = (
            copy.deepcopy(
                subtasks
            )
            + copy.deepcopy(
                remaining
            )
        )

        return self._seek(
            state=method_state,
            tasks=next_tasks,
            plan=copy.deepcopy(
                plan
            ),
            trace=next_trace,
        )

    @staticmethod
    def _failure_result(
        state: WorldState,
    ) -> PlanningResult:
        """Create a failed planning result."""

        return PlanningResult(
            success=False,
            plan=[],
            final_state=copy.deepcopy(
                state
            ),
            trace=PlanningTrace(),
            reason=(
                "No valid HTN "
                "decomposition found."
            ),
        )
