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

from mission_formalism_evaluation.bt.nodes.communication_recovery import (
    CommunicationRecoveryController,
)


class GPSTreeBuilder:
    """
    Incrementally scenario-capable Behavior Tree builder.

    Current capabilities:

        - nominal mission execution
        - GPS recovery
        - propulsion-failure handling
        - communication recovery

    Architecture:

        SCENARIO_CAPABLE_ROOT [Parallel]
        │
        ├── GPS_RECOVERY_CONTROLLER
        ├── PROPULSION_FAILURE_CONTROLLER
        ├── COMMUNICATION_RECOVERY_CONTROLLER
        │
        └── MISSION_SEQUENCE [memory=True]
            ├── TAKEOFF
            ├── NAV ...
            └── LAND

    Runtime handlers are shared across the complete mission.
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
        # Shared recovery/safety controllers
        # ==========================================================

        gps_controller = (
            GPSRecoveryController(
                name="GPS_RECOVERY_CONTROLLER",
                executor=self.executor,
                landing_duration=self.task_duration,
            )
        )

        propulsion_controller = (
            PropulsionFailureController(
                name="PROPULSION_FAILURE_CONTROLLER",
                executor=self.executor,
                landing_duration=self.task_duration,
            )
        )

        communication_controller = (
            CommunicationRecoveryController(
                name="COMMUNICATION_RECOVERY_CONTROLLER",
                executor=self.executor,
                rtl_duration=self.task_duration,
            )
        )

        # ==========================================================
        # Parallel runtime root
        # ==========================================================

        root = py_trees.composites.Parallel(
            name="SCENARIO_CAPABLE_ROOT",
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
                communication_controller,
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
