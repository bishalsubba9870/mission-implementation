"""Nominal finite-state-machine mission executor."""

from typing import Any

import rclpy

from rclpy.node import Node

from mission_formalism_evaluation.common.mission_loader import (
    load_mission,
)


class FSMExecutor(Node):
    """Execute a nominal YAML mission using an FSM."""

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
        """Build nominal mission states and transitions."""

        self.states = {
            'IDLE': None,
            'COMPLETED': None,
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

    def _handle_event(
        self,
        event: str,
    ) -> None:
        """Apply one FSM event."""

        transition_key = (
            self.current_state,
            event,
        )

        next_state = self.transitions.get(
            transition_key
        )

        if next_state is None:
            self.get_logger().error(
                f'No transition from state '
                f'{self.current_state} '
                f'for event {event}.'
            )
            return

        previous_state = (
            self.current_state
        )

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
        """Execute a state's entry action."""

        if state_name == 'COMPLETED':
            self._finish_mission()
            return

        task = self.states[
            state_name
        ]

        if task is None:
            return

        action = self._task_action(
            task
        )

        self.get_logger().info(
            f'STATE: {state_name} | '
            f'ACTION: {action}'
        )

    def _task_action(
        self,
        task: dict[str, Any],
    ) -> str:
        """Create the action label for a task."""

        task_type = str(
            task['type']
        ).lower()

        if task_type == 'takeoff':
            return 'TAKEOFF'

        if task_type == 'navigate':
            return (
                f"NAVIGATE -> "
                f"{task['waypoint']}"
            )

        if task_type == 'land':
            return 'LAND'

        raise ValueError(
            f'Unsupported task type: '
            f'{task_type}'
        )

    def _timer_callback(self) -> None:
        """Complete the active nominal state."""

        if self.mission_finished:
            return

        if self.current_state in {
            'IDLE',
            'COMPLETED',
        }:
            return

        self.get_logger().info(
            f'EVENT: TASK_COMPLETED '
            f'in {self.current_state}'
        )

        self._handle_event(
            'TASK_COMPLETED'
        )

    def _log_mission_summary(
        self,
    ) -> None:
        """Log nominal FSM structure."""

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

    def _finish_mission(self) -> None:
        """Finish nominal mission execution."""

        if self.mission_finished:
            return

        self.mission_finished = True

        self.timer.cancel()

        self.get_logger().info(
            'FSM mission completed successfully.'
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
