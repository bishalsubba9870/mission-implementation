"""GPS-capable Behavior Tree construction."""

import py_trees

from mission_formalism_evaluation.bt.nodes.gps_recovery import (
    GPSRecoveryController,
)
from mission_formalism_evaluation.bt.nodes.mission_action import (
    MissionAction,
)


class GPSTreeBuilder:
    """Build a mission tree with one shared GPS controller."""

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
        """Build GPS-capable Behavior Tree."""

        mission_sequence = self._build_mission_sequence(
            tasks
        )

        gps_controller = GPSRecoveryController(
            name='GPS_RECOVERY_CONTROLLER',
            executor=self.executor,
            landing_duration=self.task_duration,
        )

        root = py_trees.composites.Parallel(
            name='GPS_CAPABLE_ROOT',
            policy=(
                py_trees.common
                .ParallelPolicy
                .SuccessOnAll(
                    synchronise=False
                )
            ),
        )

        root.add_children(
            [
                gps_controller,
                mission_sequence,
            ]
        )

        return py_trees.trees.BehaviourTree(
            root=root
        )

    def _build_mission_sequence(
        self,
        tasks,
    ):
        """Build nominal mission sequence."""

        sequence = py_trees.composites.Sequence(
            name='MISSION_SEQUENCE',
            memory=True,
        )

        sequence.add_children(
            [
                self._create_task_node(task)
                for task in tasks
            ]
        )

        return sequence

    def _create_task_node(
        self,
        task,
    ):
        """Create one mission leaf."""

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
        """Create readable node name."""

        if task['type'] == 'navigate':
            return (
                f"NAV_{task['index']}_"
                f"{task['target']}"
            )

        return (
            f"{task['type'].upper()}_"
            f"{task['index']}"
        )
