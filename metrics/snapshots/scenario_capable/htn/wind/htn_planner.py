"""
Generic totally ordered state-based HTN planner.

The planner follows SHOP-style forward decomposition.

The planner itself contains no UAV scenario knowledge.
The same planner can therefore be reused when runtime
recovery methods are added later.

Primary basis:
    Nau et al., SHOP2, JAIR 2003.
"""

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
    """Record HTN structures used during planning."""

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
    """Result of one planning invocation."""

    success: bool

    plan: list[
        Task
    ]

    final_state: WorldState

    trace: PlanningTrace

    reason: str = ""


class HTNPlanner:
    """Totally ordered state-based HTN planner."""

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

        if not tasks:

            return (
                plan,
                state,
                trace,
            )

        task = tasks[0]

        remaining = (
            tasks[1:]
        )

        task_name = (
            task[0]
        )

        # --------------------------------------------------------
        # Primitive task
        # --------------------------------------------------------

        if self.domain.is_primitive(
            task_name
        ):

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

            next_state = (
                copy.deepcopy(
                    state
                )
            )

            operator.effect(
                next_state,
                task,
            )

            next_trace = (
                copy.deepcopy(
                    trace
                )
            )

            next_trace\
                .primitive_tasks\
                .append(
                    task_name
                )

            next_plan = (
                copy.deepcopy(
                    plan
                )
            )

            next_plan.append(
                copy.deepcopy(
                    task
                )
            )

            return self._seek(
                state=next_state,
                tasks=copy.deepcopy(
                    remaining
                ),
                plan=next_plan,
                trace=next_trace,
            )

        # --------------------------------------------------------
        # Compound task
        # --------------------------------------------------------

        if self.domain.is_compound(
            task_name
        ):

            methods = (
                self.domain
                .methods[
                    task_name
                ]
            )

            for method in methods:

                method_state = (
                    copy.deepcopy(
                        state
                    )
                )

                if not method.precondition(
                    method_state,
                    task,
                ):

                    continue

                subtasks = (
                    method.decompose(
                        method_state,
                        task,
                    )
                )

                next_trace = (
                    copy.deepcopy(
                        trace
                    )
                )

                next_trace\
                    .compound_tasks\
                    .append(
                        task_name
                    )

                next_trace\
                    .methods\
                    .append(
                        method.name
                    )

                result = self._seek(
                    state=method_state,
                    tasks=(
                        copy.deepcopy(
                            subtasks
                        )
                        + copy.deepcopy(
                            remaining
                        )
                    ),
                    plan=copy.deepcopy(
                        plan
                    ),
                    trace=next_trace,
                )

                if result is not None:

                    return result

            return None

        raise ValueError(
            "HTN task is neither "
            "primitive nor compound: "
            f"{task_name}"
        )
