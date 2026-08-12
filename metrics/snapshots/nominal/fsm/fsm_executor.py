import os
from typing import Any

import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class FSMExecutor(Node):
    """Execute a YAML mission using a finite-state machine."""

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

        package_share = get_package_share_directory(
            'mission_formalism_evaluation'
        )

        mission_path = os.path.join(
            package_share,
            'missions',
            mission_file,
        )

        self.mission_id = ''
        self.mission_name = ''
        self.tasks: list[dict[str, Any]] = []

        # FSM structures
        self.current_state = 'IDLE'
        self.states: dict[str, dict[str, Any] | None] = {}
        self.transitions: dict[tuple[str, str], str] = {}

        self.mission_finished = False

        self._load_mission(mission_path)
        self._build_fsm()

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

        self.timer = self.create_timer(
            self.state_duration,
            self._timer_callback,
        )

        # IDLE --START--> first mission state
        self._handle_event('START')

    def _load_mission(self, mission_path: str) -> None:
        """Load and validate the neutral mission YAML file."""

        self.get_logger().info(
            f'Loading mission: {mission_path}'
        )

        if not os.path.exists(mission_path):
            raise FileNotFoundError(
                f'Mission file not found: {mission_path}'
            )

        with open(
            mission_path,
            'r',
            encoding='utf-8',
        ) as file:
            data = yaml.safe_load(file)

        if not isinstance(data, dict):
            raise ValueError(
                'Mission YAML root must be a mapping.'
            )

        if 'mission' not in data or 'tasks' not in data:
            raise ValueError(
                "Mission YAML requires 'mission' and 'tasks'."
            )

        mission_data = data['mission']
        tasks = data['tasks']

        if not isinstance(tasks, list) or not tasks:
            raise ValueError(
                'Mission must contain at least one task.'
            )

        self.mission_id = str(mission_data['id'])
        self.mission_name = str(mission_data['name'])
        self.tasks = tasks

    def _build_fsm(self) -> None:
        """Generate mission states and transitions from the task list."""

        self.states = {
            'IDLE': None,
            'COMPLETED': None,
        }

        task_state_names: list[str] = []

        for index, task in enumerate(self.tasks):
            state_name = self._create_state_name(
                task,
                index,
            )

            self.states[state_name] = task
            task_state_names.append(state_name)

        # Initial transition
        self.transitions[
            ('IDLE', 'START')
        ] = task_state_names[0]

        # Sequential task transitions
        for index, state_name in enumerate(task_state_names):
            is_last_task = index == len(task_state_names) - 1

            if is_last_task:
                next_state = 'COMPLETED'
            else:
                next_state = task_state_names[index + 1]

            self.transitions[
                (state_name, 'TASK_COMPLETED')
            ] = next_state

    def _create_state_name(
        self,
        task: dict[str, Any],
        index: int,
    ) -> str:
        """Create a unique FSM state name for a mission task."""

        task_type = str(task.get('type', '')).lower()

        if task_type == 'takeoff':
            return f'TAKEOFF_{index}'

        if task_type == 'navigate':
            waypoint = task.get('waypoint')

            if waypoint is None:
                raise ValueError(
                    f'Navigate task {index} has no waypoint.'
                )

            # Index keeps repeated waypoints unique.
            return f'NAV_{index}_{waypoint}'

        if task_type == 'land':
            return f'LAND_{index}'

        raise ValueError(
            f'Unsupported task type at index {index}: '
            f'{task_type}'
        )

    def _handle_event(self, event: str) -> None:
        """Evaluate an event and perform the matching transition."""

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
                f'{self.current_state} for event {event}.'
            )
            return

        previous_state = self.current_state
        self.current_state = next_state

        self.get_logger().info(
            f'TRANSITION: {previous_state} '
            f'--[{event}]--> {next_state}'
        )

        self._enter_state(next_state)

    def _enter_state(self, state_name: str) -> None:
        """Execute the entry action of the new state."""

        if state_name == 'COMPLETED':
            self._finish_mission()
            return

        task = self.states[state_name]

        if task is None:
            return

        task_type = str(task['type']).lower()

        if task_type == 'takeoff':
            action = 'TAKEOFF'

        elif task_type == 'navigate':
            action = (
                f"NAVIGATE -> {task['waypoint']}"
            )

        elif task_type == 'land':
            action = 'LAND'

        else:
            raise ValueError(
                f'Unsupported task type: {task_type}'
            )

        self.get_logger().info(
            f'STATE: {state_name} | ACTION: {action}'
        )

    def _timer_callback(self) -> None:
        """Simulate completion of the currently active state."""

        if self.mission_finished:
            return

        if self.current_state in {
            'IDLE',
            'COMPLETED',
        }:
            return

        self.get_logger().info(
            f'EVENT: TASK_COMPLETED in '
            f'{self.current_state}'
        )

        self._handle_event('TASK_COMPLETED')

    def _finish_mission(self) -> None:
        """Stop the FSM after successful completion."""

        if self.mission_finished:
            return

        self.mission_finished = True
        self.timer.cancel()

        self.get_logger().info(
            'FSM mission completed successfully.'
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
        print(f'[FSM EXECUTOR ERROR] {error}')

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
