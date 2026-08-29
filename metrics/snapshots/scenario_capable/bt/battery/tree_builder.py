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

from mission_formalism_evaluation.bt.nodes.wind_unsafe import (
    WindUnsafeController,
)

from mission_formalism_evaluation.bt.nodes.battery_critical import (
    BatteryCriticalController,
)


class GPSTreeBuilder:

    def __init__(
        self,
        executor,
        task_duration,
    ):
        self.executor = executor
        self.task_duration = float(task_duration)

    def build(
        self,
        tasks,
    ):

        mission_sequence = py_trees.composites.Sequence(
            name="MISSION_SEQUENCE",
            memory=True,
        )

        for task in tasks:
            mission_sequence.add_child(
                MissionAction(
                    name=self.make_node_name(task),
                    task=task,
                    executor=self.executor,
                    task_duration=self.task_duration,
                )
            )

        gps_controller = GPSRecoveryController(
            name="GPS_RECOVERY_CONTROLLER",
            executor=self.executor,
            landing_duration=self.task_duration,
        )

        propulsion_controller = PropulsionFailureController(
            name="PROPULSION_FAILURE_CONTROLLER",
            executor=self.executor,
            landing_duration=self.task_duration,
        )

        communication_controller = CommunicationRecoveryController(
            name="COMMUNICATION_RECOVERY_CONTROLLER",
            executor=self.executor,
            rtl_duration=self.task_duration,
        )

        wind_controller = WindUnsafeController(
            name="WIND_UNSAFE_CONTROLLER",
            executor=self.executor,
            abort_duration=self.task_duration,
            landing_duration=self.task_duration,
        )

        battery_controller = BatteryCriticalController(
            name="BATTERY_CRITICAL_CONTROLLER",
            executor=self.executor,
            landing_duration=self.task_duration,
        )

        root = py_trees.composites.Parallel(
            name="SCENARIO_CAPABLE_ROOT",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False
            ),
        )

        root.add_children(
            [
                gps_controller,
                propulsion_controller,
                communication_controller,
                wind_controller,
                battery_controller,
                mission_sequence,
            ]
        )

        return py_trees.trees.BehaviourTree(root)

    @staticmethod
    def make_node_name(task):

        task_type = str(
            task.get("type", "task")
        ).lower()

        index = task.get("index", 0)

        if task_type == "navigate":
            target = str(
                task.get("target", "")
            )

            return f"NAV_{index}_{target}"

        return f"{task_type.upper()}_{index}"
