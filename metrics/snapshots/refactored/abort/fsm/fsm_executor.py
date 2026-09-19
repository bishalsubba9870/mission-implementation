"""FSM executor with all runtime recovery scenarios."""

from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mission_formalism_evaluation.common.mission_loader import load_mission
from mission_formalism_evaluation.common.progress_interface import publish_progress
from mission_formalism_evaluation.common.runtime_condition import parse_runtime_event


class FSMExecutor(Node):
    """Execute a mission using an event-driven finite-state machine."""

    def __init__(self) -> None:
        super().__init__('fsm_executor')

        self.declare_parameter('mission_file', 'mission_1.yaml')
        self.declare_parameter('state_duration', 1.0)

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
        self.states: dict[str, dict[str, Any] | None] = {}
        self.transitions: dict[tuple[str, str], str] = {}

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

        self._handle_event('START')

    def _build_fsm(self) -> None:
        """Build mission and cumulative recovery states."""

        self.states = {
            'IDLE': None,
            'COMPLETED': None,

            'GPS_RECOVERY': {
                'type': 'recovery',
                'action': 'WAIT_FOR_GPS_STATUS',
            },

            'COMMUNICATION_RECOVERY': {
                'type': 'recovery',
                'action': 'WAIT_FOR_COMMUNICATION',
            },

            'WIND_ABORT': {
                'type': 'recovery',
                'action': 'ABORT_MISSION',
            },

            'RETURN_TO_HOME': {
                'type': 'recovery',
                'action': 'RETURN_TO_HOME',
            },

            'LAND': {
                'type': 'recovery',
                'action': 'LAND',
            },

            'SAFE_TERMINATED': None,
        }

        task_states = [
            self._add_task_state(task, index)
            for index, task in enumerate(self.tasks)
        ]

        self.transitions[
            ('IDLE', 'START')
        ] = task_states[0]

        self._add_nominal_transitions(
            task_states
        )

        self.transitions[
            ('GPS_RECOVERY', 'GPS_RECOVERY_FAILED')
        ] = 'LAND'

        self.transitions[
            (
                'COMMUNICATION_RECOVERY',
                'COMMUNICATION_RECOVERY_FAILED',
            )
        ] = 'RETURN_TO_HOME'

        self.transitions[
            ('RETURN_TO_HOME', 'RTL_COMPLETED')
        ] = 'SAFE_TERMINATED'

        self.transitions[
            ('WIND_ABORT', 'WIND_ABORT_COMPLETED')
        ] = 'LAND'

        self.transitions[
            ('LAND', 'LAND_COMPLETED')
        ] = 'SAFE_TERMINATED'

    def _add_nominal_transitions(
        self,
        task_states: list[str],
    ) -> None:
        """Add nominal and runtime transitions."""

        runtime_transitions = {
            'GPS_LOST':
                'GPS_RECOVERY',

            'PROPULSION_FAILURE':
                'LAND',

            'COMMUNICATION_LOST':
                'COMMUNICATION_RECOVERY',

            'WIND_UNSAFE':
                'WIND_ABORT',

            'BATTERY_CRITICAL':
                'LAND',

            'ABORT_REQUESTED':
                'RETURN_TO_HOME',
        }

        for index, state_name in enumerate(
            task_states
        ):
            next_state = (
                task_states[index + 1]
                if index + 1 < len(task_states)
                else 'COMPLETED'
            )

            self.transitions[
                (state_name, 'TASK_COMPLETED')
            ] = next_state

            if not self._is_navigation_state(
                state_name
            ):
                continue

            for event, target in (
                runtime_transitions.items()
            ):
                self.transitions[
                    (state_name, event)
                ] = target

    def _add_task_state(
        self,
        task: dict[str, Any],
        index: int,
    ) -> str:
        """Create and register one task state."""

        state_name = self._create_state_name(
            task,
            index,
        )

        self.states[state_name] = task

        return state_name

    def _create_state_name(
        self,
        task: dict[str, Any],
        index: int,
    ) -> str:
        """Create a unique state name."""

        task_type = str(
            task.get('type', '')
        ).lower()

        if task_type == 'takeoff':
            return f'TAKEOFF_{index}'

        if task_type == 'navigate':
            waypoint = task.get('waypoint')

            if waypoint is None:
                raise ValueError(
                    f'Navigate task {index} has no waypoint.'
                )

            return f'NAV_{index}_{waypoint}'

        if task_type == 'land':
            return f'LAND_{index}'

        raise ValueError(
            f'Unsupported task type at index {index}: {task_type}'
        )

    def _is_navigation_state(
        self,
        state_name: str,
    ) -> bool:
        """Return whether a state is a navigation state."""

        state_data = self.states.get(
            state_name
        )

        if state_data is None:
            return False

        return (
            str(
                state_data.get('type', '')
            ).lower()
            == 'navigate'
        )

    def _handle_event(
        self,
        event: str,
    ) -> None:
        """Perform the transition associated with an event."""

        next_state = self.transitions.get(
            (
                self.current_state,
                event,
            )
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
        """Execute state-entry behavior."""

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
        """Return action text for one state."""

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
            f'Unsupported state type: {state_type}'
        )

    def _timer_callback(self) -> None:
        """Advance automatically completing states."""

        if self.mission_finished:
            return

        if self.current_state in {
            'IDLE',
            'COMPLETED',
            'SAFE_TERMINATED',
            'GPS_RECOVERY',
            'COMMUNICATION_RECOVERY',
        }:
            return

        recovery_completions = {
            'WIND_ABORT':
                'WIND_ABORT_COMPLETED',

            'RETURN_TO_HOME':
                'RTL_COMPLETED',

            'LAND':
                'LAND_COMPLETED',
        }

        recovery_event = recovery_completions.get(
            self.current_state
        )

        if recovery_event is not None:
            self._complete_recovery_state(
                recovery_event
            )
            return

        self._complete_nominal_task()

    def _complete_nominal_task(self) -> None:
        """Complete the active nominal mission task."""

        state_data = self.states.get(
            self.current_state
        )

        if state_data is None:
            return

        state_type = str(
            state_data.get('type', '')
        ).lower()

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

    def _complete_recovery_state(
        self,
        completion_event: str,
    ) -> None:
        """Complete an automatic recovery state."""

        state_name = self.current_state

        self._publish_progress(
            state_name,
            'recovery',
            'COMPLETED',
        )

        self.get_logger().info(
            f'EVENT: {completion_event} '
            f'in {state_name}'
        )

        self._handle_event(
            completion_event
        )

    def _runtime_event_callback(
        self,
        message: String,
    ) -> None:
        """Dispatch supported runtime events."""

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
                f'Runtime event not yet supported by FSM: {event}'
            )
            return

        handler()

    def _begin_navigation_recovery(
        self,
        event: str,
        remember_state: bool,
    ) -> None:
        """Interrupt navigation and enter recovery."""

        if not self._is_navigation_state(
            self.current_state
        ):
            self.get_logger().warning(
                f'{event} ignored because FSM '
                'is not in a navigation state.'
            )
            return

        if remember_state:
            self.interrupted_state = self.current_state
        else:
            self.interrupted_state = None

        self._handle_event(
            event
        )

    def _handle_gps_lost(self) -> None:
        self._begin_navigation_recovery(
            'GPS_LOST',
            remember_state=True,
        )

    def _handle_gps_available(self) -> None:
        self._resume_interrupted_state(
            expected_state='GPS_RECOVERY',
            event='GPS_AVAILABLE',
            description='GPS recovered.',
        )

    def _handle_gps_recovery_failed(
        self,
    ) -> None:
        self._fail_recovery(
            expected_state='GPS_RECOVERY',
            event='GPS_RECOVERY_FAILED',
        )

    def _handle_propulsion_failure(
        self,
    ) -> None:
        self.termination_reason = (
            'PROPULSION_FAILURE'
        )

        self._begin_navigation_recovery(
            'PROPULSION_FAILURE',
            remember_state=False,
        )

    def _handle_communication_lost(
        self,
    ) -> None:
        self._begin_navigation_recovery(
            'COMMUNICATION_LOST',
            remember_state=True,
        )

    def _handle_communication_available(
        self,
    ) -> None:
        self._resume_interrupted_state(
            expected_state='COMMUNICATION_RECOVERY',
            event='COMMUNICATION_AVAILABLE',
            description='Communication restored.',
        )

    def _handle_communication_recovery_failed(
        self,
    ) -> None:
        self._fail_recovery(
            expected_state='COMMUNICATION_RECOVERY',
            event='COMMUNICATION_RECOVERY_FAILED',
        )

    def _handle_wind_unsafe(self) -> None:
        self.termination_reason = (
            'WIND_UNSAFE'
        )

        self._begin_navigation_recovery(
            'WIND_UNSAFE',
            remember_state=False,
        )

    def _handle_battery_critical(self) -> None:
        self.termination_reason = (
            'BATTERY_CRITICAL'
        )

        self._begin_navigation_recovery(
            'BATTERY_CRITICAL',
            remember_state=False,
        )

    def _handle_abort_requested(self) -> None:
        """Abort mission and return home."""

        self.termination_reason = (
            'ABORT_REQUESTED'
        )

        self._begin_navigation_recovery(
            'ABORT_REQUESTED',
            remember_state=False,
        )

    def _fail_recovery(
        self,
        expected_state: str,
        event: str,
    ) -> None:
        """Process an unsuccessful recovery attempt."""

        if self.current_state != expected_state:
            self.get_logger().warning(
                f'{event} received outside '
                f'{expected_state}.'
            )
            return

        self.termination_reason = event
        self.interrupted_state = None

        self._handle_event(
            event
        )

    def _resume_interrupted_state(
        self,
        expected_state: str,
        event: str,
        description: str,
    ) -> None:
        """Resume the mission state interrupted by recovery."""

        if self.current_state != expected_state:
            self.get_logger().warning(
                f'{event} received outside '
                f'{expected_state}.'
            )
            return

        if self.interrupted_state is None:
            self.get_logger().error(
                'No interrupted state available to resume.'
            )
            return

        previous_state = self.current_state
        resume_state = self.interrupted_state

        self.current_state = resume_state
        self.interrupted_state = None

        self.get_logger().info(
            f'TRANSITION: {previous_state} '
            f'--[{event}]--> {resume_state}'
        )

        self.get_logger().info(
            f'{description} Resuming '
            'interrupted mission task.'
        )

        self._enter_state(
            resume_state
        )

    def _publish_progress(
        self,
        state_name: str,
        task_type: str,
        status: str,
    ) -> None:
        """Publish standardized mission progress."""

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

    def _log_mission_summary(self) -> None:
        """Log mission and FSM structure."""

        self.get_logger().info(
            f'Mission ID: {self.mission_id}'
        )

        self.get_logger().info(
            f'Mission Name: {self.mission_name}'
        )

        self.get_logger().info(
            f'Mission Items: {len(self.tasks)}'
        )

        self.get_logger().info(
            f'FSM States: {len(self.states)}'
        )

        self.get_logger().info(
            f'FSM Transitions: {len(self.transitions)}'
        )

    def _finish_mission(
        self,
        result: str,
    ) -> None:
        """Finish FSM execution."""

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
                f'Reason: {self.termination_reason}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)

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
            f'[FSM EXECUTOR ERROR] {error}'
        )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
