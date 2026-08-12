import math
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class GazeboExecutor(Node):
    """
    Shared Gazebo execution adapter for all mission formalisms.

    Mission layer decides WHAT to do:
        TAKEOFF
        NAVIGATE WPx
        RETURN_TO_HOME / RTL
        LAND

    This executor handles HOW the simulated multicopter moves.

    ROS interfaces
    --------------
    Subscribe:
        /mission/action
            std_msgs/String

        /model/x3/odometry
            nav_msgs/Odometry

    Publish:
        /X3/gazebo/command/twist
            geometry_msgs/Twist

        /gazebo_executor/status
            std_msgs/String
    """

    def __init__(self) -> None:
        super().__init__('gazebo_executor')

        # =========================================================
        # Parameters
        # =========================================================

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

        # =========================================================
        # Waypoint database
        # =========================================================

        self.waypoints: Dict[
            str,
            Tuple[float, float, float],
        ] = {}

        self._generate_waypoints()

        # =========================================================
        # Vehicle state
        # =========================================================

        self.current_position = [
            0.0,
            0.0,
            0.0,
        ]

        self.current_yaw = 0.0

        self.have_odometry = False

        # Home is captured from the first odometry message.
        self.home_ground_position: Optional[
            Tuple[float, float, float]
        ] = None

        # =========================================================
        # Mission action state
        # =========================================================

        self.current_action = 'IDLE'

        self.target_position: Optional[
            Tuple[float, float, float]
        ] = None

        self.active_waypoint = ''

        # Prevent repeated identical action commands from
        # restarting an action unnecessarily.
        self.last_received_command = ''

        # =========================================================
        # ROS publishers
        # =========================================================

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

        # =========================================================
        # ROS subscribers
        # =========================================================

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

        # =========================================================
        # Controller timer
        # =========================================================

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

    # =============================================================
    # Waypoint generation
    # =============================================================

    def _generate_waypoints(self) -> None:
        """
        Generate WP1 ... WP100.

        For now this creates a simple zigzag:

            WP1   (2,   0, 3)
            WP2   (4,   2, 3)
            WP3   (6,   0, 3)
            WP4   (8,   2, 3)
            ...
            WP100 (200, 2, 3)

        The mapping belongs to the common Gazebo execution
        environment, not to any specific mission formalism.
        """

        for index in range(
            1,
            101,
        ):

            x = float(
                index * 2
            )

            if index % 2 == 0:
                y = 2.0
            else:
                y = 0.0

            z = self.takeoff_altitude

            waypoint_name = (
                f'WP{index}'
            )

            self.waypoints[
                waypoint_name
            ] = (
                x,
                y,
                z,
            )

    # =============================================================
    # Odometry
    # =============================================================

    def _odometry_callback(
        self,
        message: Odometry,
    ) -> None:
        """
        Store current simulated position and yaw.
        """

        position = (
            message.pose.pose.position
        )

        orientation = (
            message.pose.pose.orientation
        )

        self.current_position[0] = (
            position.x
        )

        self.current_position[1] = (
            position.y
        )

        self.current_position[2] = (
            position.z
        )

        # ---------------------------------------------------------
        # Quaternion -> yaw
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # First odometry message
        # ---------------------------------------------------------

        if not self.have_odometry:

            self.have_odometry = True

            self.home_ground_position = (
                position.x,
                position.y,
                position.z,
            )

            self.get_logger().info(
                'X3 odometry received.'
            )

            self.get_logger().info(
                'Home position captured: '
                f'({position.x:.2f}, '
                f'{position.y:.2f}, '
                f'{position.z:.2f})'
            )

    # =============================================================
    # Mission action callback
    # =============================================================

    def _action_callback(
        self,
        message: String,
    ) -> None:

        command = (
            message.data
            .strip()
            .upper()
        )

        if not command:
            return

        self.get_logger().info(
            f'MISSION ACTION RECEIVED: '
            f'{command}'
        )

        if not self.have_odometry:

            self.get_logger().warning(
                'Cannot execute mission action: '
                'odometry has not been received yet.'
            )

            return

        # =========================================================
        # TAKEOFF
        # =========================================================

        if command == 'TAKEOFF':

            self.current_action = (
                'TAKEOFF'
            )

            self.target_position = (
                self.current_position[0],
                self.current_position[1],
                self.takeoff_altitude,
            )

            self.active_waypoint = ''

            self.last_received_command = (
                command
            )

            self._publish_status(
                'TAKEOFF_RUNNING'
            )

            self.get_logger().info(
                'Executing TAKEOFF -> '
                f'{self.takeoff_altitude:.2f} m'
            )

            return

        # =========================================================
        # NAVIGATE WPx
        # =========================================================

        if command.startswith(
            'NAVIGATE '
        ):

            parts = (
                command.split()
            )

            if len(parts) != 2:

                self.get_logger().error(
                    f'Invalid NAVIGATE command: '
                    f'{command}'
                )

                return

            waypoint_name = (
                parts[1]
            )

            if (
                waypoint_name
                not in self.waypoints
            ):

                self.get_logger().error(
                    f'Unknown waypoint: '
                    f'{waypoint_name}'
                )

                return

            self.current_action = (
                'NAVIGATE'
            )

            self.active_waypoint = (
                waypoint_name
            )

            self.target_position = (
                self.waypoints[
                    waypoint_name
                ]
            )

            self.last_received_command = (
                command
            )

            target_x = (
                self.target_position[0]
            )

            target_y = (
                self.target_position[1]
            )

            target_z = (
                self.target_position[2]
            )

            self.get_logger().info(
                f'Navigating to '
                f'{waypoint_name}: '
                f'({target_x:.2f}, '
                f'{target_y:.2f}, '
                f'{target_z:.2f})'
            )

            self._publish_status(
                f'NAVIGATION_RUNNING '
                f'{waypoint_name}'
            )

            return

        # =========================================================
        # RETURN TO HOME / RTL
        # =========================================================

        if command in {
            'RTL',
            'RETURN_TO_HOME',
        }:

            if (
                self.home_ground_position
                is None
            ):

                self.get_logger().error(
                    'Cannot RTL: home position '
                    'has not been captured.'
                )

                return

            home_x = (
                self.home_ground_position[0]
            )

            home_y = (
                self.home_ground_position[1]
            )

            # Return home while remaining airborne.
            home_z = (
                self.takeoff_altitude
            )

            self.current_action = (
                'RETURN_TO_HOME'
            )

            self.target_position = (
                home_x,
                home_y,
                home_z,
            )

            self.active_waypoint = ''

            self.last_received_command = (
                command
            )

            self.get_logger().info(
                'Executing RETURN_TO_HOME.'
            )

            self._publish_status(
                'RTL_RUNNING'
            )

            return

        # =========================================================
        # LAND
        # =========================================================

        if command in {
            'LAND',
            'SAFE_LAND',
            'EMERGENCY_LAND',
        }:

            self.current_action = (
                'LAND'
            )

            self.target_position = (
                self.current_position[0],
                self.current_position[1],
                self.landing_altitude,
            )

            self.active_waypoint = ''

            self.last_received_command = (
                command
            )

            self.get_logger().info(
                'Executing LAND.'
            )

            self._publish_status(
                'LAND_RUNNING'
            )

            return

        # =========================================================
        # ABORT_MISSION
        # =========================================================

        if command == 'ABORT_MISSION':

            self.current_action = (
                'IDLE'
            )

            self.target_position = None

            self.active_waypoint = ''

            self.last_received_command = (
                command
            )

            self._publish_stop()

            self.get_logger().warning(
                'Nominal mission execution aborted.'
            )

            self._publish_status(
                'ABORT_COMPLETED'
            )

            return

        # =========================================================
        # STOP
        # =========================================================

        if command == 'STOP':

            self.current_action = (
                'IDLE'
            )

            self.target_position = None

            self.active_waypoint = ''

            self.last_received_command = (
                command
            )

            self._publish_stop()

            self.get_logger().info(
                'Vehicle motion stopped.'
            )

            self._publish_status(
                'STOPPED'
            )

            return

        self.get_logger().warning(
            f'Unsupported mission action: '
            f'{command}'
        )

    # =============================================================
    # Main control loop
    # =============================================================

    def _control_loop(
        self,
    ) -> None:

        if not self.have_odometry:
            return

        if self.target_position is None:
            return

        current_x = (
            self.current_position[0]
        )

        current_y = (
            self.current_position[1]
        )

        current_z = (
            self.current_position[2]
        )

        target_x = (
            self.target_position[0]
        )

        target_y = (
            self.target_position[1]
        )

        target_z = (
            self.target_position[2]
        )

        error_world_x = (
            target_x - current_x
        )

        error_world_y = (
            target_y - current_y
        )

        error_z = (
            target_z - current_z
        )

        horizontal_distance = (
            math.sqrt(
                error_world_x ** 2
                + error_world_y ** 2
            )
        )

        distance_3d = (
            math.sqrt(
                error_world_x ** 2
                + error_world_y ** 2
                + error_z ** 2
            )
        )

        # =========================================================
        # Check target completion
        # =========================================================

        if (
            distance_3d
            <= self.waypoint_tolerance
        ):

            self._complete_current_action()

            return

        command = Twist()

        # =========================================================
        # TAKEOFF
        # =========================================================

        if (
            self.current_action
            == 'TAKEOFF'
        ):

            command.linear.z = (
                self._limited_vertical_velocity(
                    error_z
                )
            )

            self.velocity_publisher.publish(
                command
            )

            return

        # =========================================================
        # LAND
        # =========================================================

        if (
            self.current_action
            == 'LAND'
        ):

            command.linear.z = (
                self._limited_vertical_velocity(
                    error_z
                )
            )

            self.velocity_publisher.publish(
                command
            )

            return

        # =========================================================
        # NAVIGATION / RTL
        # =========================================================

        if self.current_action in {
            'NAVIGATE',
            'RETURN_TO_HOME',
        }:

            # -----------------------------------------------------
            # Convert world-frame XY error into body-frame XY
            # because MulticopterVelocityControl expects velocity
            # relative to vehicle orientation.
            # -----------------------------------------------------

            cos_yaw = math.cos(
                self.current_yaw
            )

            sin_yaw = math.sin(
                self.current_yaw
            )

            error_body_x = (
                cos_yaw
                * error_world_x
                + sin_yaw
                * error_world_y
            )

            error_body_y = (
                -sin_yaw
                * error_world_x
                + cos_yaw
                * error_world_y
            )

            if horizontal_distance > 0.01:

                command.linear.x = (
                    self.cruise_speed
                    * error_body_x
                    / horizontal_distance
                )

                command.linear.y = (
                    self.cruise_speed
                    * error_body_y
                    / horizontal_distance
                )

            command.linear.z = (
                self._limited_vertical_velocity(
                    error_z
                )
            )

            self.velocity_publisher.publish(
                command
            )

    # =============================================================
    # Action completion
    # =============================================================

    def _complete_current_action(
        self,
    ) -> None:

        completed_action = (
            self.current_action
        )

        completed_waypoint = (
            self.active_waypoint
        )

        self._publish_stop()

        self.current_action = (
            'IDLE'
        )

        self.target_position = None

        self.active_waypoint = ''

        # ---------------------------------------------------------
        # TAKEOFF
        # ---------------------------------------------------------

        if completed_action == 'TAKEOFF':

            self.get_logger().info(
                'TAKEOFF COMPLETED'
            )

            self._publish_status(
                'TAKEOFF_COMPLETED'
            )

            return

        # ---------------------------------------------------------
        # NAVIGATE
        # ---------------------------------------------------------

        if completed_action == 'NAVIGATE':

            self.get_logger().info(
                f'WAYPOINT REACHED: '
                f'{completed_waypoint}'
            )

            self._publish_status(
                f'NAVIGATION_COMPLETED '
                f'{completed_waypoint}'
            )

            return

        # ---------------------------------------------------------
        # RTL
        # ---------------------------------------------------------

        if (
            completed_action
            == 'RETURN_TO_HOME'
        ):

            self.get_logger().info(
                'RETURN TO HOME COMPLETED'
            )

            self._publish_status(
                'RTL_COMPLETED'
            )

            return

        # ---------------------------------------------------------
        # LAND
        # ---------------------------------------------------------

        if completed_action == 'LAND':

            self.get_logger().info(
                'LAND COMPLETED'
            )

            self._publish_status(
                'LAND_COMPLETED'
            )

            return

    # =============================================================
    # Velocity helper
    # =============================================================

    def _limited_vertical_velocity(
        self,
        error_z: float,
    ) -> float:

        if abs(error_z) < 0.05:
            return 0.0

        return max(
            -self.vertical_speed,
            min(
                self.vertical_speed,
                error_z,
            ),
        )

    # =============================================================
    # Publish stop
    # =============================================================

    def _publish_stop(
        self,
    ) -> None:

        command = Twist()

        self.velocity_publisher.publish(
            command
        )

    # =============================================================
    # Status publishing
    # =============================================================

    def _publish_status(
        self,
        status: str,
    ) -> None:

        message = String()

        message.data = (
            status
        )

        self.status_publisher.publish(
            message
        )


def main(
    args=None,
) -> None:

    rclpy.init(
        args=args
    )

    node = (
        GazeboExecutor()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node._publish_stop()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()
