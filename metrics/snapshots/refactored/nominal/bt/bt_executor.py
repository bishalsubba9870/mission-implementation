"""Nominal Behavior Tree mission executor."""

import json

import py_trees
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from mission_formalism_evaluation.bt.task_adapter import (
    normalize_tasks,
)
from mission_formalism_evaluation.bt.tree_builder import (
    NominalTreeBuilder,
)
from mission_formalism_evaluation.common.mission_loader import (
    load_mission,
)


class BehaviorTreeExecutor(Node):
    """Execute a nominal mission using a Behavior Tree."""

    def __init__(self) -> None:
        super().__init__('bt_executor')

        self.declare_parameter(
            'mission_file',
            'mission_1.yaml',
        )
        self.declare_parameter(
            'state_duration',
            0.1,
        )
        self.declare_parameter(
            'tick_period',
            0.02,
        )

        mission_file = (
            self.get_parameter('mission_file')
            .get_parameter_value()
            .string_value
        )

        self.state_duration = (
            self.get_parameter('state_duration')
            .get_parameter_value()
            .double_value
        )

        self.tick_period = (
            self.get_parameter('tick_period')
            .get_parameter_value()
            .double_value
        )

        (
            self.mission_id,
            self.mission_name,
            raw_tasks,
            mission_path,
        ) = load_mission(
            'mission_formalism_evaluation',
            mission_file,
        )

        self.tasks = normalize_tasks(
            raw_tasks
        )

        self.completed_tasks = 0
        self.completed_navigation_tasks = 0
        self.finished = False

        self.progress_publisher = self.create_publisher(
            String,
            '/mission/progress',
            10,
        )

        builder = NominalTreeBuilder(
            executor=self,
            task_duration=self.state_duration,
        )

        self.tree = builder.build(
            self.tasks
        )

        self.get_logger().info(
            f'Loading mission: {mission_path}'
        )

        self._log_mission_summary()
        self._log_tree_structure()

        self.timer = self.create_timer(
            self.tick_period,
            self._tick_tree,
        )

        self.get_logger().info(
            'Behavior Tree execution started.'
        )

    def _tick_tree(self) -> None:
        """Tick the tree and process root status."""

        if self.finished:
            return

        self.tree.tick()

        status = self.tree.root.status

        if status == py_trees.common.Status.SUCCESS:
            self._finish_mission(
                'COMPLETED'
            )

        elif status == py_trees.common.Status.FAILURE:
            self._finish_mission(
                'FAILED'
            )

    def _finish_mission(
        self,
        status: str,
    ) -> None:
        """Finish BT mission execution."""

        if self.finished:
            return

        self.finished = True
        self.timer.cancel()

        self._publish_mission_status(
            status
        )

        if status == 'COMPLETED':
            self.get_logger().info(
                'MISSION COMPLETED'
            )

            self.get_logger().info(
                f'Completed tasks: '
                f'{self.completed_tasks}'
            )

            self.get_logger().info(
                f'Completed navigation tasks: '
                f'{self.completed_navigation_tasks}'
            )
            return

        self.get_logger().error(
            'MISSION FAILED'
        )

    def on_task_started(
        self,
        task,
    ) -> None:
        """Handle leaf start."""

        self.get_logger().info(
            f'BT NODE RUNNING: '
            f'{self._node_name(task)}'
        )

        self.get_logger().info(
            f'ACTION: '
            f'{self._action_description(task)}'
        )

        self._publish_task_progress(
            task,
            'RUNNING',
        )

    def on_task_completed(
        self,
        task,
    ) -> None:
        """Handle successful leaf completion."""

        self.completed_tasks += 1

        if task['type'] == 'navigate':
            self.completed_navigation_tasks += 1

        self.get_logger().info(
            f'BT NODE SUCCESS: '
            f'{self._node_name(task)}'
        )

        self.get_logger().info(
            f'Completed tasks: '
            f'{self.completed_tasks}/'
            f'{len(self.tasks)}'
        )

        self._publish_task_progress(
            task,
            'COMPLETED',
        )

    @staticmethod
    def _node_name(
        task,
    ) -> str:
        """Return BT node name."""

        if task['type'] == 'navigate':
            return (
                f"NAV_{task['index']}_"
                f"{task['target']}"
            )

        return (
            f"{task['type'].upper()}_"
            f"{task['index']}"
        )

    @staticmethod
    def _action_description(
        task,
    ) -> str:
        """Return human-readable task action."""

        descriptions = {
            'takeoff': 'TAKEOFF',
            'hover': 'HOVER',
            'inspect': 'INSPECT',
            'land': 'LAND',
            'rtl': 'RETURN_TO_HOME',
        }

        if task['type'] == 'navigate':
            return (
                f"NAVIGATE -> "
                f"{task['target']}"
            )

        return descriptions.get(
            task['type'],
            task['name'],
        )

    def _publish_task_progress(
        self,
        task,
        status: str,
    ) -> None:
        """Publish task progress."""

        payload = {
            'mission':
                self.mission_id,

            'formalism':
                'BT',

            'task_index':
                task['index'],

            'task_id':
                task['id'],

            'task_name':
                self._node_name(task),

            'task_type':
                task['type'],

            'task_status':
                status,

            'target':
                task['target'],

            'completed_task_count':
                self.completed_tasks,

            'completed_navigation_count':
                self.completed_navigation_tasks,
        }

        self._publish_payload(
            payload
        )

    def _publish_mission_status(
        self,
        status: str,
    ) -> None:
        """Publish overall mission status."""

        payload = {
            'mission':
                self.mission_id,

            'formalism':
                'BT',

            'mission_status':
                status,

            'completed_task_count':
                self.completed_tasks,

            'completed_navigation_count':
                self.completed_navigation_tasks,
        }

        self._publish_payload(
            payload
        )

    def _publish_payload(
        self,
        payload: dict,
    ) -> None:
        """Publish one JSON progress message."""

        message = String()

        message.data = json.dumps(
            payload,
            separators=(',', ':'),
        )

        self.progress_publisher.publish(
            message
        )

    def _log_mission_summary(self) -> None:
        """Log mission information."""

        self.get_logger().info(
            f'Mission ID: {self.mission_id}'
        )

        self.get_logger().info(
            f'Mission name: {self.mission_name}'
        )

        self.get_logger().info(
            f'Mission items: {len(self.tasks)}'
        )

    def _log_tree_structure(self) -> None:
        """Log structural BT measurements."""

        nodes = list(
            self.tree.root.iterate()
        )

        leaf_nodes = sum(
            1
            for node in nodes
            if not node.children
        )

        composite_nodes = (
            len(nodes)
            - leaf_nodes
        )

        self.get_logger().info(
            f'BT total nodes: {len(nodes)}'
        )

        self.get_logger().info(
            f'BT composite nodes: '
            f'{composite_nodes}'
        )

        self.get_logger().info(
            f'BT leaf/action nodes: '
            f'{leaf_nodes}'
        )

        self.get_logger().info(
            'Root: MISSION_SEQUENCE'
        )

        self.get_logger().info(
            'Root type: Sequence(memory=True)'
        )

        self.get_logger().info(
            '\n'
            + py_trees.display.unicode_tree(
                self.tree.root,
                show_status=False,
            )
        )


def main(args=None) -> None:
    rclpy.init(
        args=args
    )

    node = None

    try:
        node = BehaviorTreeExecutor()
        rclpy.spin(node)

    except (
        FileNotFoundError,
        KeyError,
        TypeError,
        ValueError,
    ) as error:
        print(
            f'[BT EXECUTOR ERROR] {error}'
        )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
