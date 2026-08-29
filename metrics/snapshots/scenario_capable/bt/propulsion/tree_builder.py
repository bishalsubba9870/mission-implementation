#!/usr/bin/env python3

import py_trees

from mission_formalism_evaluation.bt.nodes.mission_action import (
    MissionAction,
)

from mission_formalism_evaluation.bt.nodes.gps_recovery import (
    GPSRecoveryController,
)

from mission_formalism_evaluation.bt.nodes.propulsion_failure import (
    PropulsionFailureController,
)


class GPSTreeBuilder:
    """
    Incrementally scenario-capable Behavior Tree builder.

    Current capability stage:

        - nominal mission execution
        - GPS recovery
        - propulsion-failure emergency landing

    Architecture:

        GPS_PROPULSION_ROOT [Parallel]
        │
        ├── GPS_RECOVERY_CONTROLLER
        │
        ├── PROPULSION_FAILURE_CONTROLLER
        │
        └── MISSION_SEQUENCE [Sequence, memory=True]
            ├── TAKEOFF
            ├── NAV ...
            └── LAND

    The safety/recovery behaviours are shared across the mission
    rather than duplicated for every mission action.
    """

    def __init__(
        self,
        executor,
        task_duration,
    ):
        self.executor = executor

        self.task_duration = float(
            task_duration
        )

    def build(
        self,
        tasks,
    ):
        # ==========================================================
        # Mission sequence
        # ==========================================================

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
                    task_duration=self.task_duration,
                )
            )

        # ==========================================================
        # GPS recovery controller
        # ==========================================================

        gps_controller = (
            GPSRecoveryController(
                name="GPS_RECOVERY_CONTROLLER",
                executor=self.executor,
                landing_duration=self.task_duration,
            )
        )

        # ==========================================================
        # Propulsion failure controller
        # ==========================================================

        propulsion_controller = (
            PropulsionFailureController(
                name="PROPULSION_FAILURE_CONTROLLER",
                executor=self.executor,
                landing_duration=self.task_duration,
            )
        )

        # ==========================================================
        # Parallel root
        # ==========================================================
        #
        # The mission and runtime recovery/safety controllers are
        # ticked concurrently.
        #
        # SuccessOnAll means normal mission completion requires all
        # branches to be successful.
        # ==========================================================

        root = py_trees.composites.Parallel(
            name="GPS_PROPULSION_ROOT",
            policy=(
                py_trees.common.ParallelPolicy.SuccessOnAll(
                    synchronise=False
                )
            ),
        )

        root.add_children(
            [
                gps_controller,
                propulsion_controller,
                mission_sequence,
            ]
        )

        return py_trees.trees.BehaviourTree(
            root
        )

    @staticmethod
    def make_node_name(
        task,
    ):
        """
        Generate stable BT node names.
        """

        task_type = str(
            task.get(
                "type",
                "task",
            )
        ).lower()

        index = task.get(
            "index",
            0,
        )

        if task_type == "navigate":
            target = str(
                task.get(
                    "target",
                    "",
                )
            )

            return (
                f"NAV_{index}_{target}"
            )

        return (
            f"{task_type.upper()}_{index}"
        )
