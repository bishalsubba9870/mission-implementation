"""Nominal Behavior Tree construction."""

import py_trees

from mission_formalism_evaluation.bt.nodes.mission_action import (
    MissionAction,
)


class NominalTreeBuilder:
    """Build the nominal sequential mission tree."""

    def __init__(
        self,
        executor,
        task_duration,
    ):
        self.executor = executor
        self.task_duration = task_duration

    def build(
        self,
        tasks,
    ):
        """Build and return the nominal Behavior Tree."""

        root = py_trees.composites.Sequence(
            name='MISSION_SEQUENCE',
            memory=True,
        )

        children = [
            self._create_task_node(task)
            for task in tasks
        ]

        root.add_children(
            children
        )

        return py_trees.trees.BehaviourTree(
            root=root
        )

    def _create_task_node(
        self,
        task,
    ):
        """Create one mission-action leaf."""

        return MissionAction(
            name=self.make_node_name(task),
            task=task,
            executor=self.executor,
            duration=self.task_duration,
        )

    @staticmethod
    def make_node_name(
        task,
    ):
        """Create a readable BT node name."""

        if task['type'] == 'navigate':
            return (
                f"NAV_{task['index']}_"
                f"{task['target']}"
            )

        return (
            f"{task['type'].upper()}_"
            f"{task['index']}"
        )
