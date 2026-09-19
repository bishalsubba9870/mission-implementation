"""FSM mission executor with GPS and propulsion recovery."""

from typing import Any

import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from mission_formalism_evaluation.common.mission_loader import (
    load_mission,
)

from mission_formalism_evaluation.common.progress_interface import (
    publish_progress,
)

from mission_formalism_evaluation.common.runtime_condition import (
    parse_runtime_event,
)


class FSMExecutor(Node):
    """Execute a mission using an FSM with runtime recovery."""

    def __init__(self) -> None:
        super().__init__('fsm_executor')

        self.declare_parameter(
            'mission_file',
            'mission_1.yaml',
        )

        self.declare_parameter(
            'state_duration',
            1.0,
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

        (
            self.mission_id,
            self.mission_name,
            self.tasks,
            mission_path,
        ) = load_mission(
            'mission_formalism_evaluation',
            mission_file,
        )

        self.current_state = 'IDLE'

        self.states: dict[
            str,
            dict[str, Any] | None,
        ] = {}

        self.transitions: dict[
            tuple[str, str],
            str,
        ] = {}

        self.mission_finished = False

        self.completed_navigation_tasks = 0

        self.interrupted_state: str | None = None

        self.termination_reason = ''

        self.progress_publisher = self.create_publisher(
            String,
            '/mission/progress',
            10,
        )

        self.event_subscription = self.create_subscription(
            String,
            '/mission/event',
            self._runtime_event_callback,
            10,
        )

        self.get_logger().info(
            f'Loading mission: {mission_path}'
        )

        self._build_fsm()
        self._log_mission_summary()

        self.timer = self.create_timer(
            self.state_duration,
            self._timer_callback,
        )

        self._handle_event(
            'START'
        )

    def _build_fsm(self) -> None:
        """Build mission, GPS recovery, and propulsion handling."""

        self.states = {
            'IDLE': None,

            'COMPLETED': None,

            'GPS_RECOVERY': {
                'type': 'recovery',
                'action': 'WAIT_FOR_GPS_STATUS',
            },

            'LAND': {
                'type': 'recovery',
                'action': 'LAND',
            },

            'SAFE_TERMINATED': None,
        }

        task_states = [
            self._add_task_state(
                task,
                index,
            )
            for index, task in enumerate(
                self.tasks
            )
        ]

        self.transitions[
            ('IDLE', 'START')
        ] = task_states[0]

        for index, state_name in enumerate(
            task_states
        ):
            if index + 1 < len(task_states):
                next_state = task_states[
                    index + 1
                ]
            else:
                next_state = 'COMPLETED'

            self.transitions[
                (
                    state_name,
                    'TASK_COMPLETED',
                )
            ] = next_state

            if self._is_navigation_state(
                state_name
            ):
                self.transitions[
                    (
                        state_name,
                        'GPS_LOST',
                    )
                ] = 'GPS_RECOVERY'

                self.transitions[
                    (
                        state_name,
                        'PROPULSION_FAILURE',
                    )
                ] = 'LAND'

        self.transitions[
            (
                'GPS_RECOVERY',
                'GPS_RECOVERY_FAILED',
            )
        ] = 'LAND'

        self.transitions[
            (
                'LAND',
                'LAND_COMPLETED',
            )
        ] = 'SAFE_TERMINATED'

    def _add_task_state(
        self,
        task: dict[str, Any],
        index: int,
    ) -> str:
        """Create and register one mission state."""

        state_name = self._create_state_name(
            task,
            index,
        )

        self.states[
            state_name
        ] = task

        return state_name

    def _create_state_name(
        self,
        task: dict[str, Any],
        index: int,
    ) -> str:
        """Create a unique FSM state name."""

        task_type = str(
            task.get(
                'type',
                '',
            )
        ).lower()

        if task_type == 'takeoff':
            return f'TAKEOFF_{index}'

        if task_type == 'navigate':
            waypoint = task.get(
                'waypoint'
            )

            if waypoint is None:
                raise ValueError(
                    f'Navigate task {index} '
                    'has no waypoint.'
                )

            return (
                f'NAV_{index}_{waypoint}'
            )

        if task_type == 'land':
            return f'LAND_{index}'

        raise ValueError(
            f'Unsupported task type at '
            f'index {index}: {task_type}'
        )

    def _is_navigation_state(
        self,
        state_name: str,
    ) -> bool:
        """Return whether a state represents navigation."""

        state_data = self.states.get(
            state_name
        )

        if state_data is None:
            return False

        return (
            str(
                state_data.get(
                    'type',
                    '',
                )
            ).lower()
            == 'navigate'
        )

    def _handle_event(
        self,
        event: str,
    ) -> None:
        """Apply one FSM transition."""

        transition_key = (
            self.current_state,
            event,
        )

        next_state = self.transitions.get(
            transition_key
        )

        if next_state is None:
            self.get_logger().warning(
                f'No transition from state '
                f'{self.current_state} '
                f'for event {event}.'
            )
            return

        previous_state = self.current_state

        self.current_state = next_state

        self.get_logger().info(
            f'TRANSITION: {previous_state} '
            f'--[{event}]--> {next_state}'
        )

        self._enter_state(
            next_state
        )

    def _enter_state(
        self,
        state_name: str,
    ) -> None:
        """Execute the entry behavior of a state."""

        if state_name == 'COMPLETED':
            self.get_logger().info(
                'STATE: COMPLETED'
            )

            self._finish_mission(
                'SUCCESS'
            )
            return

        if state_name == 'SAFE_TERMINATED':
            self.get_logger().info(
                'STATE: SAFE_TERMINATED'
            )

            self._finish_mission(
                'SAFE_TERMINATED'
            )
            return

        state_data = self.states[
            state_name
        ]

        if state_data is None:
            return

        state_type = str(
            state_data['type']
        ).lower()

        action = self._state_action(
            state_data
        )

        self.get_logger().info(
            f'STATE: {state_name} '
            f'| ACTION: {action}'
        )

        self._publish_progress(
            state_name,
            state_type,
            'RUNNING',
        )

    def _state_action(
        self,
        state_data: dict[str, Any],
    ) -> str:
        """Return action label for a state."""

        state_type = str(
            state_data['type']
        ).lower()

        if state_type == 'takeoff':
            return 'TAKEOFF'

        if state_type == 'navigate':
            return (
                f"NAVIGATE -> "
                f"{state_data['waypoint']}"
            )

        if state_type == 'land':
            return 'LAND'

        if state_type == 'recovery':
            return str(
                state_data['action']
            )

        raise ValueError(
            f'Unsupported state type: '
            f'{state_type}'
        )

    def _timer_callback(self) -> None:
        """Complete the currently active FSM state."""

        if self.mission_finished:
            return

        if self.current_state in {
            'IDLE',
            'COMPLETED',
            'SAFE_TERMINATED',
        }:
            return

        if self.current_state == 'GPS_RECOVERY':
            return

        state_data = self.states.get(
            self.current_state
        )

        if state_data is None:
            return

        state_type = str(
            state_data.get(
                'type',
                '',
            )
        ).lower()

        if self.current_state == 'LAND':
            self._complete_recovery_land()
            return

        self._publish_progress(
            self.current_state,
            state_type,
            'COMPLETED',
        )

        if state_type == 'navigate':
            self.completed_navigation_tasks += 1

        self.get_logger().info(
            f'EVENT: TASK_COMPLETED '
            f'in {self.current_state}'
        )

        self._handle_event(
            'TASK_COMPLETED'
        )

    def _complete_recovery_land(
        self,
    ) -> None:
        """Complete emergency landing."""

        self._publish_progress(
            'LAND',
            'recovery',
            'COMPLETED',
        )

        self.get_logger().info(
            'EVENT: LAND_COMPLETED in LAND'
        )

        self._handle_event(
            'LAND_COMPLETED'
        )

    def _runtime_event_callback(
        self,
        message: String,
    ) -> None:
        """Dispatch runtime events."""

        if self.mission_finished:
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
        }

        handler = handlers.get(
            event
        )

        if handler is None:
            self.get_logger().warning(
                f'Runtime event not yet '
                f'supported by FSM: {event}'
            )
            return

        handler()

    def _handle_gps_lost(
        self,
    ) -> None:
        """Enter GPS recovery from navigation."""

        if not self._is_navigation_state(
            self.current_state
        ):
            self.get_logger().warning(
                'GPS_LOST ignored because '
                'FSM is not in a navigation state.'
            )
            return

        self.interrupted_state = (
            self.current_state
        )

        self._handle_event(
            'GPS_LOST'
        )

    def _handle_gps_available(
        self,
    ) -> None:
        """Resume interrupted task after GPS recovery."""

        if self.current_state != 'GPS_RECOVERY':
            self.get_logger().warning(
                'GPS_AVAILABLE received '
                'outside GPS_RECOVERY.'
            )
            return

        if self.interrupted_state is None:
            self.get_logger().error(
                'No interrupted state '
                'available to resume.'
            )
            return

        previous_state = self.current_state

        resume_state = self.interrupted_state

        self.current_state = resume_state

        self.interrupted_state = None

        self.get_logger().info(
            f'TRANSITION: {previous_state} '
            f'--[GPS_AVAILABLE]--> '
            f'{resume_state}'
        )

        self.get_logger().info(
            'GPS recovered. Resuming '
            'interrupted mission task.'
        )

        self._enter_state(
            resume_state
        )

    def _handle_gps_recovery_failed(
        self,
    ) -> None:
        """Handle failed GPS recovery."""

        if self.current_state != 'GPS_RECOVERY':
            self.get_logger().warning(
                'GPS_RECOVERY_FAILED received '
                'outside GPS_RECOVERY.'
            )
            return

        self.termination_reason = (
            'GPS_RECOVERY_FAILED'
        )

        self.interrupted_state = None

        self._handle_event(
            'GPS_RECOVERY_FAILED'
        )

    def _handle_propulsion_failure(
        self,
    ) -> None:
        """Handle propulsion failure during navigation."""

        if not self._is_navigation_state(
            self.current_state
        ):
            self.get_logger().warning(
                'PROPULSION_FAILURE ignored because '
                'FSM is not in a navigation state.'
            )
            return

        self.termination_reason = (
            'PROPULSION_FAILURE'
        )

        self.interrupted_state = None

        self._handle_event(
            'PROPULSION_FAILURE'
        )

    def _publish_progress(
        self,
        state_name: str,
        task_type: str,
        status: str,
    ) -> None:
        """Publish mission progress."""

        publish_progress(
            publisher=self.progress_publisher,
            mission_id=self.mission_id,
            state_name=state_name,
            task_type=task_type,
            status=status,
            completed_navigation_tasks=(
                self.completed_navigation_tasks
            ),
        )

    def _log_mission_summary(
        self,
    ) -> None:
        """Log FSM mission structure."""

        self.get_logger().info(
            f'Mission ID: {self.mission_id}'
        )

        self.get_logger().info(
            f'Mission Name: '
            f'{self.mission_name}'
        )

        self.get_logger().info(
            f'Mission Items: '
            f'{len(self.tasks)}'
        )

        self.get_logger().info(
            f'FSM States: '
            f'{len(self.states)}'
        )

        self.get_logger().info(
            f'FSM Transitions: '
            f'{len(self.transitions)}'
        )

    def _finish_mission(
        self,
        result: str,
    ) -> None:
        """Terminate FSM execution."""

        if self.mission_finished:
            return

        self.mission_finished = True

        self.timer.cancel()

        if result == 'SUCCESS':
            self.get_logger().info(
                'FSM mission completed successfully.'
            )
            return

        self.get_logger().warning(
            'FSM mission terminated safely.'
        )

        if self.termination_reason:
            self.get_logger().warning(
                f'Reason: '
                f'{self.termination_reason}'
            )


def main(args=None) -> None:
    rclpy.init(
        args=args
    )

    node = None

    try:
        node = FSMExecutor()
        rclpy.spin(node)

    except (
        FileNotFoundError,
        KeyError,
        TypeError,
        ValueError,
    ) as error:
        print(
            f'[FSM EXECUTOR ERROR] '
            f'{error}'
        )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
