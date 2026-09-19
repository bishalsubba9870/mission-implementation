import math
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class GazeboExecutor(Node):
    """Shared Gazebo execution adapter for all mission formalisms."""

    def __init__(self) -> None:
        super().__init__('gazebo_executor')

        self.declare_parameter(
            'cruise_speed',
            1.0,
        )

        self.declare_parameter(
            'vertical_speed',
            0.7,
        )

        self.declare_parameter(
            'waypoint_tolerance',
            0.30,
        )

        self.declare_parameter(
            'takeoff_altitude',
            3.0,
        )

        self.declare_parameter(
            'landing_altitude',
            0.20,
        )

        self.cruise_speed = float(
            self.get_parameter(
                'cruise_speed'
            ).value
        )

        self.vertical_speed = float(
            self.get_parameter(
                'vertical_speed'
            ).value
        )

        self.waypoint_tolerance = float(
            self.get_parameter(
                'waypoint_tolerance'
            ).value
        )

        self.takeoff_altitude = float(
            self.get_parameter(
                'takeoff_altitude'
            ).value
        )

        self.landing_altitude = float(
            self.get_parameter(
                'landing_altitude'
            ).value
        )

        self.waypoints: Dict[
            str,
            Tuple[float, float, float],
        ] = {
            'WP1': (
                2.0,
                0.0,
                self.takeoff_altitude,
            ),
            'WP2': (
                4.0,
                2.0,
                self.takeoff_altitude,
            ),
            'WP3': (
                4.0,
                -2.0,
                self.takeoff_altitude,
            ),
            'WP4': (
                6.0,
                0.0,
                self.takeoff_altitude,
            ),
            'WP5': (
                8.0,
                2.0,
                self.takeoff_altitude,
            ),
            'WP6': (
                8.0,
                -2.0,
                self.takeoff_altitude,
            ),
        }

        self.current_position = [
            0.0,
            0.0,
            0.0,
        ]

        self.current_yaw = 0.0
        self.have_odometry = False

        self.home_ground_position: Optional[
            Tuple[float, float, float]
        ] = None

        self.current_action = 'IDLE'

        self.target_position: Optional[
            Tuple[float, float, float]
        ] = None

        self.active_waypoint = ''
        self.last_received_command = ''

        self.velocity_publisher = self.create_publisher(
            Twist,
            '/X3/gazebo/command/twist',
            10,
        )

        self.status_publisher = self.create_publisher(
            String,
            '/gazebo_executor/status',
            10,
        )

        self.action_subscription = self.create_subscription(
            String,
            '/mission/action',
            self._action_callback,
            10,
        )

        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/model/x3/odometry',
            self._odometry_callback,
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self._control_loop,
        )

        self.get_logger().info(
            'Gazebo executor started.'
        )

        self.get_logger().info(
            'Waiting for odometry on '
            '/model/x3/odometry'
        )

        self.get_logger().info(
            'Listening for mission actions on '
            '/mission/action'
        )

        self._log_waypoints()

    def _log_waypoints(self) -> None:
        """Log configured Gazebo waypoint coordinates."""

        self.get_logger().info(
            'Configured Gazebo waypoints:'
        )

        for name, position in self.waypoints.items():
            self.get_logger().info(
                f'{name}: '
                f'x={position[0]:.1f}, '
                f'y={position[1]:.1f}, '
                f'z={position[2]:.1f}'
            )

    def _odometry_callback(
        self,
        message: Odometry,
    ) -> None:
        """Update current multicopter position."""

        position = (
            message.pose.pose.position
        )

        self.current_position = [
            float(position.x),
            float(position.y),
            float(position.z),
        ]

        orientation = (
            message.pose.pose.orientation
        )

        siny_cosp = (
            2.0
            * (
                orientation.w
                * orientation.z
                + orientation.x
                * orientation.y
            )
        )

        cosy_cosp = (
            1.0
            - 2.0
            * (
                orientation.y
                * orientation.y
                + orientation.z
                * orientation.z
            )
        )

        self.current_yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        if not self.have_odometry:
            self.have_odometry = True

            self.home_ground_position = (
                self.current_position[0],
                self.current_position[1],
                self.current_position[2],
            )

            self.get_logger().info(
                'Initial odometry received.'
            )

            self.get_logger().info(
                'Home position captured: '
                f'x={self.home_ground_position[0]:.2f}, '
                f'y={self.home_ground_position[1]:.2f}, '
                f'z={self.home_ground_position[2]:.2f}'
            )

    def _action_callback(
        self,
        message: String,
    ) -> None:
        """Receive mission-level action commands."""

        command = (
            message.data
            .strip()
            .upper()
        )

        if not command:
            return

        if command == self.last_received_command:
            return

        self.last_received_command = command

        self.get_logger().info(
            f'Received mission action: {command}'
        )

        if command == 'TAKEOFF':
            self._start_takeoff()
            return

        if command.startswith('NAVIGATE'):
            self._start_navigation(
                command
            )
            return

        if command in {
            'RTL',
            'RETURN_TO_HOME',
        }:
            self._start_return_home()
            return

        if command == 'LAND':
            self._start_landing()
            return

        if command == 'STOP':
            self._stop_vehicle()
            return

        self.get_logger().warning(
            f'Unknown mission action: {command}'
        )

    def _start_takeoff(self) -> None:
        """Start takeoff to configured altitude."""

        if not self.have_odometry:
            self.get_logger().warning(
                'Cannot TAKEOFF: no odometry yet.'
            )
            self.last_received_command = ''
            return

        self.current_action = 'TAKEOFF'

        self.target_position = (
            self.current_position[0],
            self.current_position[1],
            self.takeoff_altitude,
        )

        self.active_waypoint = ''

        self._publish_status(
            'TAKEOFF_RUNNING'
        )

    def _start_navigation(
        self,
        command: str,
    ) -> None:
        """Start navigation to a named waypoint."""

        parts = command.split()

        if len(parts) < 2:
            self.get_logger().warning(
                f'Invalid navigation command: {command}'
            )
            self.last_received_command = ''
            return

        waypoint = parts[-1]

        if waypoint not in self.waypoints:
            self.get_logger().error(
                f'Unknown waypoint: {waypoint}'
            )

            self._publish_status(
                f'NAVIGATE_FAILED {waypoint}'
            )

            self.last_received_command = ''
            return

        self.current_action = 'NAVIGATE'
        self.active_waypoint = waypoint

        self.target_position = (
            self.waypoints[
                waypoint
            ]
        )

        self.get_logger().info(
            f'Navigating to {waypoint}: '
            f'{self.target_position}'
        )

        self._publish_status(
            f'NAVIGATE_RUNNING {waypoint}'
        )

    def _start_return_home(self) -> None:
        """Return horizontally to the recorded home location."""

        if self.home_ground_position is None:
            self.get_logger().warning(
                'Cannot RTL: home position unavailable.'
            )
            self.last_received_command = ''
            return

        self.current_action = 'RTL'

        self.active_waypoint = ''

        self.target_position = (
            self.home_ground_position[0],
            self.home_ground_position[1],
            self.takeoff_altitude,
        )

        self._publish_status(
            'RTL_RUNNING'
        )

    def _start_landing(self) -> None:
        """Start vertical landing at the current XY position."""

        if not self.have_odometry:
            self.get_logger().warning(
                'Cannot LAND: no odometry yet.'
            )
            self.last_received_command = ''
            return

        self.current_action = 'LAND'

        self.active_waypoint = ''

        self.target_position = (
            self.current_position[0],
            self.current_position[1],
            self.landing_altitude,
        )

        self._publish_status(
            'LAND_RUNNING'
        )

    def _control_loop(self) -> None:
        """Execute the currently active motion command."""

        if not self.have_odometry:
            return

        if self.current_action == 'IDLE':
            return

        if self.target_position is None:
            return

        dx = (
            self.target_position[0]
            - self.current_position[0]
        )

        dy = (
            self.target_position[1]
            - self.current_position[1]
        )

        dz = (
            self.target_position[2]
            - self.current_position[2]
        )

        distance_3d = math.sqrt(
            dx * dx
            + dy * dy
            + dz * dz
        )

        if (
            distance_3d
            <= self.waypoint_tolerance
        ):
            self._complete_action()
            return

        command = Twist()

        horizontal_distance = math.sqrt(
            dx * dx
            + dy * dy
        )

        if horizontal_distance > 0.05:
            horizontal_scale = min(
                self.cruise_speed
                / horizontal_distance,
                1.0,
            )

            command.linear.x = (
                dx
                * horizontal_scale
            )

            command.linear.y = (
                dy
                * horizontal_scale
            )

        if abs(dz) > 0.05:
            command.linear.z = max(
                -self.vertical_speed,
                min(
                    self.vertical_speed,
                    dz,
                ),
            )

        self.velocity_publisher.publish(
            command
        )

    def _complete_action(self) -> None:
        """Stop motion and report action completion."""

        completed_action = (
            self.current_action
        )

        completed_waypoint = (
            self.active_waypoint
        )

        self._publish_zero_velocity()

        self.current_action = 'IDLE'
        self.target_position = None
        self.active_waypoint = ''

        if completed_action == 'TAKEOFF':
            self.get_logger().info(
                'Takeoff completed.'
            )

            self._publish_status(
                'TAKEOFF_COMPLETED'
            )

        elif completed_action == 'NAVIGATE':
            self.get_logger().info(
                f'Waypoint reached: '
                f'{completed_waypoint}'
            )

            self._publish_status(
                f'NAVIGATE_COMPLETED '
                f'{completed_waypoint}'
            )

        elif completed_action == 'RTL':
            self.get_logger().info(
                'Return-to-home completed.'
            )

            self._publish_status(
                'RTL_COMPLETED'
            )

        elif completed_action == 'LAND':
            self.get_logger().info(
                'Landing completed.'
            )

            self._publish_status(
                'LAND_COMPLETED'
            )

        self.last_received_command = ''

    def _stop_vehicle(self) -> None:
        """Stop the simulated multicopter."""

        self._publish_zero_velocity()

        self.current_action = 'IDLE'
        self.target_position = None
        self.active_waypoint = ''

        self._publish_status(
            'STOPPED'
        )

        self.last_received_command = ''

    def _publish_zero_velocity(self) -> None:
        """Publish zero velocity."""

        command = Twist()

        self.velocity_publisher.publish(
            command
        )

    def _publish_status(
        self,
        status: str,
    ) -> None:
        """Publish Gazebo execution status."""

        message = String()
        message.data = status

        self.status_publisher.publish(
            message
        )

        self.get_logger().info(
            f'Gazebo status: {status}'
        )


def main(
    args=None,
) -> None:
    """Run Gazebo executor node."""

    rclpy.init(
        args=args
    )

    node = GazeboExecutor()

    try:
        rclpy.spin(
            node
        )
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero_velocity()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
