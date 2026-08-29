#!/usr/bin/env python3

import py_trees

from mission_formalism_evaluation.bt.nodes.gps_recovery import (
    GPSRecoveryController,
)

from mission_formalism_evaluation.bt.nodes.mission_action import (
    MissionAction,
)


class GPSTreeBuilder:
    """
    Build the GPS-capable Behavior Tree.

    Structure:

        GPS_CAPABLE_ROOT              Parallel
        ├── GPS_RECOVERY_CONTROLLER
        └── MISSION_SEQUENCE          Sequence
            ├── task 1
            ├── task 2
            ├── ...
            └── task N

    The GPS recovery controller is shared across the complete
    mission instead of being duplicated for each waypoint.
    """

    def __init__(
        self,
        executor,
        task_duration,
    ):

        self.executor = executor

        self.task_duration = (
            task_duration
        )

    def build(
        self,
        tasks,
    ):

        # ----------------------------------------------------------
        # Mission sequence
        # ----------------------------------------------------------

        mission_sequence = (
            py_trees.composites.Sequence(
                name="MISSION_SEQUENCE",
                memory=True,
            )
        )

        for task in tasks:

            mission_sequence.add_child(

                MissionAction(
                    name=self.make_node_name(
                        task
                    ),
                    task=task,
                    executor=self.executor,
                    duration=self.task_duration,
                )
            )

        # ----------------------------------------------------------
        # Shared GPS recovery node
        # ----------------------------------------------------------

        gps_recovery = (
            GPSRecoveryController(
                name=(
                    "GPS_RECOVERY_CONTROLLER"
                ),
                executor=self.executor,
                landing_duration=(
                    self.task_duration
                ),
            )
        )

        # ----------------------------------------------------------
        # Parallel root
        #
        # Both children are ticked:
        #
        # GPS controller:
        #     SUCCESS when GPS is normal
        #     RUNNING during GPS recovery
        #
        # Mission:
        #     RUNNING while mission executes
        #     SUCCESS when mission completes
        #
        # Therefore root succeeds only when all required work
        # is complete.
        # ----------------------------------------------------------

        root = (
            py_trees.composites.Parallel(
                name="GPS_CAPABLE_ROOT",
                policy=(
                    py_trees.common
                    .ParallelPolicy
                    .SuccessOnAll(
                        synchronise=False
                    )
                ),
            )
        )

        root.add_children(
            [
                gps_recovery,
                mission_sequence,
            ]
        )

        return (
            py_trees.trees.BehaviourTree(
                root=root
            )
        )

    @staticmethod
    def make_node_name(
        task,
    ):

        if (
            task["type"]
            == "navigate"
        ):

            return (
                f"NAV_"
                f"{task['index']}_"
                f"{task['target']}"
            )

        return (
            f"{task['type'].upper()}_"
            f"{task['index']}"
        )
