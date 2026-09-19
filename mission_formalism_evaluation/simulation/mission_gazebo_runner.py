import os

import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class MissionGazeboRunner(Node):
    """Dispatch mission YAML tasks to the shared Gazebo executor."""

    def __init__(self) -> None:
        super().__init__('mission_gazebo_runner')

        self.declare_parameter(
            'mission_file',
            'mission_1.yaml',
        )

        mission_file = (
            self.get_parameter('mission_file')
            .get_parameter_value()
            .string_value
        )

        package_share = get_package_share_directory(
            'mission_formalism_evaluation'
        )

        mission_path = os.path.join(
            package_share,
            'missions',
            mission_file,
        )

        with open(
            mission_path,
            'r',
            encoding='utf-8',
        ) as file:
            data = yaml.safe_load(file)

        self.mission_id = data['mission']['id']
        self.mission_name = data['mission']['name']
        self.tasks = data['tasks']

        self.current_index = 0
        self.expected_status = ''
        self.started = False
        self.finished = False

        self.action_publisher = self.create_publisher(
            String,
            '/mission/action',
            10,
        )

        self.status_subscription = self.create_subscription(
            String,
            '/gazebo_executor/status',
            self._status_callback,
            10,
        )

        # Wait until gazebo_executor is actually connected.
        self.connection_timer = self.create_timer(
            0.5,
            self._wait_for_executor,
        )

        self.get_logger().info(
            f'Loaded {self.mission_id}: '
            f'{self.mission_name}'
        )

        self.get_logger().info(
            'Waiting for Gazebo executor...'
        )

    def _wait_for_executor(self) -> None:
        """Wait until /mission/action has a subscriber."""

        if self.started:
            return

        subscriber_count = (
            self.action_publisher
            .get_subscription_count()
        )

        if subscriber_count < 1:
            return

        self.started = True
        self.connection_timer.cancel()

        self.get_logger().info(
            'Gazebo executor connected.'
        )

        self.get_logger().info(
            f'Starting {self.mission_id}: '
            f'{self.mission_name}'
        )

        self._dispatch_current_task()

    def _dispatch_current_task(self) -> None:
        """Publish the current mission task."""

        if self.finished:
            return

        if self.current_index >= len(self.tasks):
            self.finished = True

            self.get_logger().info(
                'Gazebo mission completed successfully.'
            )

            return

        task = self.tasks[
            self.current_index
        ]

        task_type = str(
            task.get('type', '')
        ).strip().lower()

        if task_type == 'takeoff':
            action = 'TAKEOFF'
            self.expected_status = (
                'TAKEOFF_COMPLETED'
            )

        elif task_type == 'navigate':
            waypoint = str(
                task.get('waypoint', '')
            ).strip().upper()

            if not waypoint:
                self.get_logger().error(
                    'Navigate task has no waypoint.'
                )
                return

            action = (
                f'NAVIGATE {waypoint}'
            )

            self.expected_status = (
                f'NAVIGATE_COMPLETED '
                f'{waypoint}'
            )

        elif task_type == 'land':
            action = 'LAND'
            self.expected_status = (
                'LAND_COMPLETED'
            )

        else:
            self.get_logger().error(
                f'Unsupported task type: '
                f'{task_type}'
            )
            return

        message = String()
        message.data = action

        self.action_publisher.publish(
            message
        )

        self.get_logger().info(
            f'[{self.current_index + 1}/'
            f'{len(self.tasks)}] '
            f'Published: {action}'
        )

        self.get_logger().info(
            f'Waiting for: '
            f'{self.expected_status}'
        )

    def _status_callback(
        self,
        message: String,
    ) -> None:
        """Advance after Gazebo completes the current action."""

        if self.finished:
            return

        status = (
            message.data
            .strip()
            .upper()
        )

        self.get_logger().info(
            f'Status received: {status}'
        )

        if status != self.expected_status:
            return

        self.get_logger().info(
            f'Completed: {status}'
        )

        self.current_index += 1
        self.expected_status = ''

        self._dispatch_current_task()


def main(args=None) -> None:
    """Run the Gazebo mission runner."""

    rclpy.init(args=args)

    node = MissionGazeboRunner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
