"""
Generic forward-chaining rule engine for Expert System mission execution.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping


FactMap = Mapping[str, Any]

Condition = Callable[
    [FactMap],
    bool,
]

Action = Callable[
    [FactMap],
    dict[str, Any],
]


@dataclass(frozen=True)
class Rule:
    """Production rule."""

    name: str
    priority: int
    condition: Condition
    action: Action


class WorkingMemory:
    """Dynamic Expert System facts."""

    def __init__(
        self,
        initial_facts: Mapping[str, Any] | None = None,
    ) -> None:

        self._facts: dict[
            str,
            Any,
        ] = dict(
            initial_facts or {}
        )

        self._revision = 0

    @property
    def revision(
        self,
    ) -> int:

        return self._revision

    def snapshot(
        self,
    ) -> dict[str, Any]:

        return dict(
            self._facts
        )

    def get(
        self,
        key: str,
        default: Any = None,
    ) -> Any:

        return self._facts.get(
            key,
            default,
        )

    def set(
        self,
        key: str,
        value: Any,
    ) -> bool:

        if (
            key in self._facts
            and self._facts[key] == value
        ):
            return False

        self._facts[key] = value
        self._revision += 1

        return True

    def update(
        self,
        updates: Mapping[str, Any],
    ) -> bool:

        changed = False

        for key, value in updates.items():

            if (
                key not in self._facts
                or self._facts[key] != value
            ):
                self._facts[key] = value
                changed = True

        if changed:
            self._revision += 1

        return changed


@dataclass(frozen=True)
class InferenceResult:
    """Result of one inference cycle."""

    cycle: int
    conflict_set: list[str]
    selected_rule: str | None
    priority: int | None
    updates: dict[str, Any]
    working_memory_changed: bool
    halted: bool
    halt_reason: str = ""


class RuleEngine:
    """Forward-chaining production-rule engine."""

    def __init__(
        self,
        rules: list[Rule],
        working_memory: WorkingMemory | None = None,
    ) -> None:

        if not rules:
            raise ValueError(
                "Knowledge Base must contain at least one rule."
            )

        self.rules = list(
            rules
        )

        self.working_memory = (
            working_memory
            if working_memory is not None
            else WorkingMemory()
        )

        self.cycle_count = 0
        self.fire_count = 0

        self._validate_rules()

    # ================================================================
    # Knowledge Base validation
    # ================================================================

    def _validate_rules(
        self,
    ) -> None:

        names = set()

        for rule in self.rules:

            if not rule.name:
                raise ValueError(
                    "Production rule name cannot be empty."
                )

            if rule.name in names:
                raise ValueError(
                    f"Duplicate production rule: {rule.name}"
                )

            if not isinstance(
                rule.priority,
                int,
            ):
                raise TypeError(
                    f"Rule '{rule.name}' priority must be an integer."
                )

            names.add(
                rule.name
            )

    # ================================================================
    # Inference cycle
    # ================================================================

    def match(
        self,
    ) -> list[Rule]:

        facts = (
            self.working_memory.snapshot()
        )

        return [
            rule
            for rule in self.rules
            if rule.condition(
                facts
            )
        ]

    def select(
        self,
        conflict_set: list[Rule],
    ) -> Rule | None:

        if not conflict_set:
            return None

        return min(
            conflict_set,
            key=lambda rule:
                rule.priority,
        )

    def fire(
        self,
        rule: Rule,
    ) -> tuple[
        dict[str, Any],
        bool,
    ]:

        facts = (
            self.working_memory.snapshot()
        )

        updates = rule.action(
            facts
        )

        if updates is None:
            updates = {}

        if not isinstance(
            updates,
            dict,
        ):
            raise TypeError(
                f"Rule '{rule.name}' action must return a dictionary."
            )

        changed = (
            self.working_memory.update(
                updates
            )
        )

        self.fire_count += 1

        return (
            updates,
            changed,
        )

    def inference_cycle(
        self,
    ) -> InferenceResult:

        self.cycle_count += 1

        conflict_set = (
            self.match()
        )

        selected_rule = (
            self.select(
                conflict_set
            )
        )

        conflict_names = [
            rule.name
            for rule
            in conflict_set
        ]

        if selected_rule is None:

            return InferenceResult(
                cycle=self.cycle_count,
                conflict_set=conflict_names,
                selected_rule=None,
                priority=None,
                updates={},
                working_memory_changed=False,
                halted=True,
                halt_reason="NO_MATCHING_RULE",
            )

        updates, changed = (
            self.fire(
                selected_rule
            )
        )

        return InferenceResult(
            cycle=self.cycle_count,
            conflict_set=conflict_names,
            selected_rule=selected_rule.name,
            priority=selected_rule.priority,
            updates=updates,
            working_memory_changed=changed,
            halted=not changed,
            halt_reason=(
                ""
                if changed
                else "NO_STATE_CHANGE"
            ),
        )

    # ================================================================
    # External fact updates
    # ================================================================

    def update_facts(
        self,
        updates: Mapping[str, Any],
    ) -> bool:

        return (
            self.working_memory.update(
                updates
            )
        )
