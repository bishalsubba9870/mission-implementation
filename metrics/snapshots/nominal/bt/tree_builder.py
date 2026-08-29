#!/usr/bin/env python3

import py_trees

from mission_formalism_evaluation.bt.nodes.mission_action import MissionAction


class NominalTreeBuilder:
    """
    Builds the nominal mission Behavior Tree.

    Structure:

        MISSION_SEQUENCE
        ├── Task 1
        ├── Task 2
        ├── ...
        └── Task N

    No runtime fault or recovery logic is included here.
    """

    def __init__(
        self,
        executor,
        task_duration,
    ):
        self.executor = executor
        self.task_duration = task_duration

    def build(self, tasks):
        root = py_trees.composites.Sequence(
            name="MISSION_SEQUENCE",
            memory=True,
        )

        for task in tasks:
            root.add_child(
                MissionAction(
                    name=self.make_node_name(task),
                    task=task,
                    executor=self.executor,
                    duration=self.task_duration,
                )
            )

        return py_trees.trees.BehaviourTree(
            root=root
        )

    @staticmethod
    def make_node_name(task):
        if task["type"] == "navigate":
            return (
                f"NAV_{task['index']}_"
                f"{task['target']}"
            )

        return (
            f"{task['type'].upper()}_"
            f"{task['index']}"
        )
