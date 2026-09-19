"""Generic totally ordered state-based HTN planner."""

from __future__ import annotations

import copy

from dataclasses import dataclass, field

from mission_formalism_evaluation.htn.htn_domain import (
    HTNDomain,
    Task,
    WorldState,
)


@dataclass
class PlanningTrace:
    """Record HTN decomposition decisions."""

    primitive_tasks: list[str] = field(
        default_factory=list
    )

    compound_tasks: list[str] = field(
        default_factory=list
    )

    methods: list[str] = field(
        default_factory=list
    )


@dataclass
class PlanningResult:
    """Result returned by the HTN planner."""

    success: bool
    plan: list[Task]
    final_state: WorldState
    trace: PlanningTrace
    reason: str = ""


@dataclass
class SearchNode:
    """One pending HTN search state."""

    state: WorldState
    tasks: list[Task]
    plan: list[Task]
    trace: PlanningTrace


class HTNPlanner:
    """Totally ordered forward-decomposition HTN planner."""

    def __init__(
        self,
        domain: HTNDomain,
    ) -> None:

        self.domain = domain

    def plan(
        self,
        initial_state: WorldState,
        tasks: list[Task],
    ) -> PlanningResult:
        """Generate a primitive plan for the task network."""

        initial_node = SearchNode(
            state=copy.deepcopy(
                initial_state
            ),
            tasks=copy.deepcopy(
                tasks
            ),
            plan=[],
            trace=PlanningTrace(),
        )

        result = self._search(
            initial_node
        )

        if result is None:

            return self._failure_result(
                initial_state
            )

        return PlanningResult(
            success=True,
            plan=result.plan,
            final_state=result.state,
            trace=result.trace,
        )

    def _search(
        self,
        initial_node: SearchNode,
    ) -> SearchNode | None:
        """Search with an explicit stack."""

        stack = [
            initial_node
        ]

        while stack:

            node = stack.pop()

            result = self._advance_node(
                node,
                stack,
            )

            if result is not None:

                return result

        return None

    def _advance_node(
        self,
        node: SearchNode,
        stack: list[SearchNode],
    ) -> SearchNode | None:
        """Advance one search branch until completion or branching."""

        while node.tasks:

            task = node.tasks[0]
            remaining = node.tasks[1:]

            task_name = task[0]

            if self.domain.is_primitive(
                task_name
            ):

                if not self._apply_primitive(
                    node,
                    task,
                    remaining,
                ):

                    return None

                continue

            if self.domain.is_compound(
                task_name
            ):

                self._expand_compound(
                    node,
                    task,
                    remaining,
                    stack,
                )

                return None

            raise ValueError(
                f"Unknown HTN task: {task_name}"
            )

        return node

    def _apply_primitive(
        self,
        node: SearchNode,
        task: Task,
        remaining: list[Task],
    ) -> bool:
        """Apply one primitive operator."""

        operator = (
            self.domain
            .operators[
                task[0]
            ]
        )

        if not operator.precondition(
            node.state,
            task,
        ):

            return False

        next_state = copy.deepcopy(
            node.state
        )

        operator.effect(
            next_state,
            task,
        )

        node.state = next_state

        node.plan.append(
            copy.deepcopy(
                task
            )
        )

        node.trace.primitive_tasks.append(
            task[0]
        )

        node.tasks = remaining

        return True

    def _expand_compound(
        self,
        node: SearchNode,
        task: Task,
        remaining: list[Task],
        stack: list[SearchNode],
    ) -> None:
        """Push applicable method alternatives onto the search stack."""

        applicable_nodes = []

        methods = (
            self.domain
            .methods.get(
                task[0],
                [],
            )
        )

        for method in methods:

            candidate = (
                self._method_candidate(
                    node,
                    task,
                    remaining,
                    method,
                )
            )

            if candidate is not None:

                applicable_nodes.append(
                    candidate
                )

        for candidate in reversed(
            applicable_nodes
        ):

            stack.append(
                candidate
            )

    @staticmethod
    def _method_candidate(
        node: SearchNode,
        task: Task,
        remaining: list[Task],
        method,
    ) -> SearchNode | None:
        """Create one applicable method branch."""

        method_state = copy.deepcopy(
            node.state
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
            node.trace
        )

        next_trace.compound_tasks.append(
            task[0]
        )

        next_trace.methods.append(
            method.name
        )

        return SearchNode(
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
                node.plan
            ),
            trace=next_trace,
        )

    @staticmethod
    def _failure_result(
        initial_state: WorldState,
    ) -> PlanningResult:
        """Create a failed planning result."""

        return PlanningResult(
            success=False,
            plan=[],
            final_state=copy.deepcopy(
                initial_state
            ),
            trace=PlanningTrace(),
            reason=(
                "No valid HTN decomposition "
                "was found."
            ),
        )
