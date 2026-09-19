"""GPS, propulsion, communication, wind, battery, and abort-capable Behavior Tree executor."""

import json

import py_trees
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from mission_formalism_evaluation.bt.task_adapter import (
    normalize_tasks,
)
from mission_formalism_evaluation.bt.tree_builder import (
    RecoveryTreeBuilder,
)
from mission_formalism_evaluation.common.mission_loader import (
    load_mission,
)
from mission_formalism_evaluation.common.runtime_condition import (
    parse_runtime_event,
)


class BehaviorTreeExecutor(Node):
    """Execute a mission using a reactive Behavior Tree."""

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
        self.current_task = None

        self.finished = False
        self.safe_terminated = False
        self.termination_reason = None

        self.gps_recovery_active = False
        self.gps_recovery_result = None
        self.gps_emergency_landing = False

        self.propulsion_failure_active = False

        self.communication_loss_active = False
        self.communication_recovery_result = None
        self.communication_rtl_active = False

        self.wind_unsafe_active = False
        self.wind_safe_landing_active = False

        self.battery_critical_active = False

        self.abort_requested_active = False
        self.abort_rtl_active = False

        self.progress_publisher = self.create_publisher(
            String,
            '/mission/progress',
            10,
        )

        self.event_subscription = self.create_subscription(
            String,
            '/mission/event',
            self._event_callback,
            10,
        )

        builder = RecoveryTreeBuilder(
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
            'GPS + propulsion + communication + wind + battery + abort '
            'Behavior Tree started.'
        )

    def _event_callback(
        self,
        message: String,
    ) -> None:
        """Dispatch supported runtime events."""

        if self.finished:
            return

        event = parse_runtime_event(
            message.data
        )

        if not event:
            return

        self.get_logger().warning(
            f'RUNTIME EVENT RECEIVED: {event}'
        )

        handlers = {
            'GPS_LOST':
                self._handle_gps_lost,

            'GPS_AVAILABLE':
                self._handle_gps_available,

            'GPS_RECOVERY_FAILED':
                self._handle_gps_recovery_failed,

            'PROPULSION_FAILURE':
                self._handle_propulsion_failure,

            'COMMUNICATION_LOST':
                self._handle_communication_lost,

            'COMMUNICATION_AVAILABLE':
                self._handle_communication_available,

            'COMMUNICATION_RECOVERY_FAILED':
                self._handle_communication_recovery_failed,

            'WIND_UNSAFE':
                self._handle_wind_unsafe,

            'BATTERY_CRITICAL':
                self._handle_battery_critical,

            'ABORT_REQUESTED':
                self._handle_abort_requested,
        }

        handler = handlers.get(
            event
        )

        if handler is None:
            self.get_logger().warning(
                f'Event not supported in current '
                f'BT stage: {event}'
            )
            return

        handler()

    def _handle_gps_lost(self) -> None:
        """Start GPS recovery."""

        if not self._navigation_active():
            self.get_logger().warning(
                'GPS_LOST ignored because '
                'navigation is not active.'
            )
            return

        if self.gps_recovery_active:
            return

        self.gps_recovery_active = True
        self.gps_recovery_result = None

        self.get_logger().warning(
            f'GPS LOST during '
            f'{self.node_name(self.current_task)}'
        )

        self.get_logger().info(
            'BT GPS RECOVERY ACTIVE'
        )

    def _handle_gps_available(self) -> None:
        """Store successful GPS recovery."""

        self._set_gps_result(
            'GPS_AVAILABLE'
        )

    def _handle_gps_recovery_failed(
        self,
    ) -> None:
        """Store failed GPS recovery."""

        self._set_gps_result(
            'GPS_RECOVERY_FAILED'
        )

    def _set_gps_result(
        self,
        result: str,
    ) -> None:
        """Store GPS recovery result."""

        if not self.gps_recovery_active:
            self.get_logger().warning(
                f'{result} ignored because '
                'GPS recovery is not active.'
            )
            return

        self.gps_recovery_result = result

    def _handle_propulsion_failure(
        self,
    ) -> None:
        """Activate propulsion-failure controller."""

        if self.propulsion_failure_active:
            return

        self.propulsion_failure_active = True

        self.get_logger().error(
            'PROPULSION FAILURE DETECTED'
        )

        self.get_logger().warning(
            'Normal mission execution interrupted.'
        )

    def _handle_communication_lost(
        self,
    ) -> None:
        """Start communication recovery."""

        if not self._navigation_active():
            self.get_logger().warning(
                'COMMUNICATION_LOST ignored because '
                'navigation is not active.'
            )
            return

        if self.communication_loss_active:
            return

        self.communication_loss_active = True
        self.communication_recovery_result = None

        self.get_logger().warning(
            f'COMMUNICATION LOST during '
            f'{self.node_name(self.current_task)}'
        )

        self.get_logger().info(
            'BT COMMUNICATION RECOVERY ACTIVE'
        )

    def _handle_communication_available(
        self,
    ) -> None:
        """Store successful communication recovery."""

        self._set_communication_result(
            'COMMUNICATION_AVAILABLE'
        )

    def _handle_communication_recovery_failed(
        self,
    ) -> None:
        """Store failed communication recovery."""

        self._set_communication_result(
            'COMMUNICATION_RECOVERY_FAILED'
        )

    def _set_communication_result(
        self,
        result: str,
    ) -> None:
        """Store communication recovery result."""

        if not self.communication_loss_active:
            self.get_logger().warning(
                f'{result} ignored because '
                'communication recovery is not active.'
            )
            return

        self.communication_recovery_result = result

    def _handle_wind_unsafe(
        self,
    ) -> None:
        """Activate unsafe-wind controller."""

        if self.wind_unsafe_active:
            return

        self.wind_unsafe_active = True

        self.get_logger().warning(
            'UNSAFE WIND DETECTED'
        )

        self.get_logger().warning(
            'Normal mission execution will be aborted.'
        )

    def _handle_battery_critical(
        self,
    ) -> None:
        """Activate critical-battery controller."""

        if self.battery_critical_active:
            return

        self.battery_critical_active = True

        self.get_logger().warning(
            'CRITICAL BATTERY DETECTED'
        )

        self.get_logger().warning(
            'Normal mission execution interrupted.'
        )

    def _handle_abort_requested(
        self,
    ) -> None:
        """Activate abort-request controller."""

        if self.abort_requested_active:
            return

        self.abort_requested_active = True

        self.get_logger().warning(
            'ABORT REQUEST RECEIVED'
        )

        self.get_logger().warning(
            'Normal mission execution will be aborted.'
        )

    def _navigation_active(self) -> bool:
        """Return whether current task is navigation."""

        return (
            self.current_task is not None
            and self.current_task['type']
            == 'navigate'
        )

    def _tick_tree(self) -> None:
        """Tick tree and process mission result."""

        if self.finished:
            return

        self.tree.tick()

        if self.safe_terminated:
            self._finish_mission(
                'SAFE_TERMINATED'
            )
            return

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
        """Finish BT execution."""

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

        if status == 'SAFE_TERMINATED':
            self.get_logger().warning(
                'MISSION SAFE TERMINATED'
            )

            self.get_logger().warning(
                f'Reason: '
                f'{self.termination_reason}'
            )
            return

        self.get_logger().error(
            'MISSION FAILED'
        )

    def on_task_started(
        self,
        task,
    ) -> None:
        """Handle mission leaf start."""

        self.current_task = task

        self.get_logger().info(
            f'BT NODE RUNNING: '
            f'{self.node_name(task)}'
        )

        self.get_logger().info(
            f'ACTION: '
            f'{self.action_description(task)}'
        )

        self._publish_task_progress(
            task,
            'RUNNING',
        )

    def on_task_resumed(
        self,
        task,
    ) -> None:
        """Handle resumed mission leaf."""

        self.current_task = task

        self.get_logger().info(
            f'BT NODE RESUMED: '
            f'{self.node_name(task)}'
        )

        self._publish_task_progress(
            task,
            'RUNNING',
        )

    def on_task_paused(
        self,
        task,
        reason,
    ) -> None:
        """Handle paused mission leaf."""

        self.get_logger().warning(
            f'BT NODE PAUSED: '
            f'{self.node_name(task)} '
            f'| Reason: {reason}'
        )

    def on_gps_recovered(self) -> None:
        """Report successful GPS recovery."""

        if self.current_task is None:
            return

        self.get_logger().info(
            f'GPS RECOVERED -> resume '
            f'{self.node_name(self.current_task)}'
        )

    def on_communication_recovered(
        self,
    ) -> None:
        """Report successful communication recovery."""

        if self.current_task is None:
            return

        self.get_logger().info(
            f'COMMUNICATION RESTORED -> resume '
            f'{self.node_name(self.current_task)}'
        )

    def on_task_completed(
        self,
        task,
    ) -> None:
        """Handle successful task completion."""

        self.completed_tasks += 1

        if task['type'] == 'navigate':
            self.completed_navigation_tasks += 1

        self.get_logger().info(
            f'BT NODE SUCCESS: '
            f'{self.node_name(task)}'
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

    def mark_safe_terminated(
        self,
        reason,
    ) -> None:
        """Mark mission safely terminated."""

        self.safe_terminated = True
        self.termination_reason = reason
        self.current_task = None

    @staticmethod
    def node_name(
        task,
    ) -> str:
        """Return readable BT node name."""

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
    def action_description(
        task,
    ) -> str:
        """Return readable action text."""

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
        """Publish injector-compatible task progress."""

        payload = {
            'mission_id':
                self.mission_id,

            'formalism':
                'BT',

            'task_index':
                task['index'],

            'task_id':
                task['id'],

            'task_name':
                self.node_name(task),

            'task_type':
                task['type'],

            'task_status':
                status,

            'target':
                task['target'],

            'completed_tasks':
                self.completed_tasks,

            'completed_navigation_tasks':
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
            'mission_id':
                self.mission_id,

            'formalism':
                'BT',

            'mission_status':
                status,

            'completed_tasks':
                self.completed_tasks,

            'completed_navigation_tasks':
                self.completed_navigation_tasks,
        }

        self._publish_payload(
            payload
        )

    def _publish_payload(
        self,
        payload: dict,
    ) -> None:
        """Publish JSON payload."""

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
            f'Mission ID: '
            f'{self.mission_id}'
        )

        self.get_logger().info(
            f'Mission name: '
            f'{self.mission_name}'
        )

        self.get_logger().info(
            f'Mission items: '
            f'{len(self.tasks)}'
        )

    def _log_tree_structure(self) -> None:
        """Log BT structural measurements."""

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
            f'BT total nodes: '
            f'{len(nodes)}'
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
            f'[BT EXECUTOR ERROR] '
            f'{error}'
        )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
