import json
import os
from typing import Any

import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class FSMExecutor(Node):
    """Execute a YAML mission using a finite-state machine."""

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

        self.current_state = 'IDLE'

        self.states: dict[
            str,
            dict[str, Any] | None
        ] = {}

        self.transitions: dict[
            tuple[str, str],
            str
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

        self._load_mission(
            mission_path
        )

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

        self._handle_event(
            'START'
        )

    def _load_mission(
        self,
        mission_path: str,
    ) -> None:
        """Load and validate mission YAML."""

        self.get_logger().info(
            f'Loading mission: {mission_path}'
        )

        if not os.path.exists(
            mission_path
        ):
            raise FileNotFoundError(
                f'Mission file not found: '
                f'{mission_path}'
            )

        with open(
            mission_path,
            'r',
            encoding='utf-8',
        ) as file:
            data = yaml.safe_load(
                file
            )

        if not isinstance(
            data,
            dict,
        ):
            raise ValueError(
                'Mission YAML root must be a mapping.'
            )

        if (
            'mission' not in data
            or 'tasks' not in data
        ):
            raise ValueError(
                "Mission YAML requires "
                "'mission' and 'tasks'."
            )

        mission_data = data[
            'mission'
        ]

        tasks = data[
            'tasks'
        ]

        if not isinstance(
            mission_data,
            dict,
        ):
            raise ValueError(
                "'mission' must be a mapping."
            )

        if (
            not isinstance(tasks, list)
            or not tasks
        ):
            raise ValueError(
                'Mission must contain '
                'at least one task.'
            )

        self.mission_id = str(
            mission_data['id']
        )

        self.mission_name = str(
            mission_data['name']
        )

        self.tasks = tasks

    def _build_fsm(self) -> None:
        """Build nominal mission plus GPS and propulsion handling."""

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

        task_state_names: list[str] = []

        for index, task in enumerate(
            self.tasks
        ):
            state_name = (
                self._create_state_name(
                    task,
                    index,
                )
            )

            self.states[
                state_name
            ] = task

            task_state_names.append(
                state_name
            )

        self.transitions[
            (
                'IDLE',
                'START',
            )
        ] = task_state_names[0]

        for index, state_name in enumerate(
            task_state_names
        ):
            is_last_task = (
                index
                == len(task_state_names) - 1
            )

            if is_last_task:
                next_state = 'COMPLETED'
            else:
                next_state = (
                    task_state_names[
                        index + 1
                    ]
                )

            self.transitions[
                (
                    state_name,
                    'TASK_COMPLETED',
                )
            ] = next_state

            state_data = self.states[
                state_name
            ]

            if (
                state_data is not None
                and str(
                    state_data.get(
                        'type',
                        '',
                    )
                ).lower()
                == 'navigate'
            ):
                # GPS fault transition
                self.transitions[
                    (
                        state_name,
                        'GPS_LOST',
                    )
                ] = 'GPS_RECOVERY'

                # Propulsion-failure transition
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
                    f'has no waypoint.'
                )

            return (
                f'NAV_{index}_{waypoint}'
            )

        if task_type == 'land':
            return f'LAND_{index}'

        raise ValueError(
            f'Unsupported task type '
            f'at index {index}: '
            f'{task_type}'
        )

    def _handle_event(
        self,
        event: str,
    ) -> None:
        """Perform the transition associated with an event."""

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

        previous_state = (
            self.current_state
        )

        self.current_state = (
            next_state
        )

        self.get_logger().info(
            f'TRANSITION: '
            f'{previous_state} '
            f'--[{event}]--> '
            f'{next_state}'
        )

        self._enter_state(
            next_state
        )

    def _enter_state(
        self,
        state_name: str,
    ) -> None:
        """Execute the entry action of a state."""

        if state_name == 'COMPLETED':
            self.get_logger().info(
                'STATE: COMPLETED'
            )

            self._finish_mission(
                result='SUCCESS'
            )
            return

        if state_name == 'SAFE_TERMINATED':
            self.get_logger().info(
                'STATE: SAFE_TERMINATED'
            )

            self._finish_mission(
                result='SAFE_TERMINATED'
            )
            return

        state_data = self.states[
            state_name
        ]

        if state_data is None:
            return

        state_type = str(
            state_data[
                'type'
            ]
        ).lower()

        if state_type == 'takeoff':
            action = 'TAKEOFF'

        elif state_type == 'navigate':
            action = (
                f"NAVIGATE -> "
                f"{state_data['waypoint']}"
            )

        elif state_type == 'land':
            action = 'LAND'

        elif state_type == 'recovery':
            action = str(
                state_data[
                    'action'
                ]
            )

        else:
            raise ValueError(
                f'Unsupported state type: '
                f'{state_type}'
            )

        self.get_logger().info(
            f'STATE: {state_name} '
            f'| ACTION: {action}'
        )

        self._publish_progress(
            state_name=state_name,
            task_type=state_type,
            status='RUNNING',
        )

    def _timer_callback(
        self,
    ) -> None:
        """Simulate completion of the active state."""

        if self.mission_finished:
            return

        if self.current_state in {
            'IDLE',
            'COMPLETED',
            'SAFE_TERMINATED',
        }:
            return

        if (
            self.current_state
            == 'GPS_RECOVERY'
        ):
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

            self._publish_progress(
                state_name='LAND',
                task_type='recovery',
                status='COMPLETED',
            )

            self.get_logger().info(
                'EVENT: LAND_COMPLETED in LAND'
            )

            self._handle_event(
                'LAND_COMPLETED'
            )

            return

        self._publish_progress(
            state_name=self.current_state,
            task_type=state_type,
            status='COMPLETED',
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

    def _runtime_event_callback(
        self,
        message: String,
    ) -> None:
        """Receive runtime events from the scenario injector."""

        if self.mission_finished:
            return

        event = self._extract_runtime_event(
            message.data
        )

        if not event:
            return

        self.get_logger().warning(
            f'RUNTIME EVENT RECEIVED: '
            f'{event}'
        )

        # =========================================================
        # GPS LOST
        # =========================================================
        if event == 'GPS_LOST':

            state_data = self.states.get(
                self.current_state
            )

            if (
                state_data is None
                or str(
                    state_data.get(
                        'type',
                        '',
                    )
                ).lower()
                != 'navigate'
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

            return

        # =========================================================
        # GPS AVAILABLE
        # =========================================================
        if event == 'GPS_AVAILABLE':

            if (
                self.current_state
                != 'GPS_RECOVERY'
            ):
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

            previous_state = (
                self.current_state
            )

            resume_state = (
                self.interrupted_state
            )

            self.current_state = (
                resume_state
            )

            self.interrupted_state = None

            self.get_logger().info(
                f'TRANSITION: '
                f'{previous_state} '
                f'--[GPS_AVAILABLE]--> '
                f'{resume_state}'
            )

            self.get_logger().info(
                'GPS recovered. '
                'Resuming interrupted mission task.'
            )

            self._enter_state(
                resume_state
            )

            return

        # =========================================================
        # GPS RECOVERY FAILED
        # =========================================================
        if event == 'GPS_RECOVERY_FAILED':

            if (
                self.current_state
                != 'GPS_RECOVERY'
            ):
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

            return

        # =========================================================
        # PROPULSION FAILURE
        # =========================================================
        if event == 'PROPULSION_FAILURE':

            state_data = self.states.get(
                self.current_state
            )

            if (
                state_data is None
                or str(
                    state_data.get(
                        'type',
                        '',
                    )
                ).lower()
                != 'navigate'
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

            return

        self.get_logger().warning(
            f'Runtime event not yet supported '
            f'by FSM: {event}'
        )

    def _extract_runtime_event(
        self,
        raw_data: str,
    ) -> str:
        """Parse JSON or plain-string runtime events."""

        try:
            parsed = json.loads(
                raw_data
            )

            if isinstance(
                parsed,
                dict,
            ):
                return str(
                    parsed.get(
                        'event',
                        '',
                    )
                ).strip().upper()

        except json.JSONDecodeError:
            pass

        return (
            raw_data
            .strip()
            .upper()
        )

    def _publish_progress(
        self,
        state_name: str,
        task_type: str,
        status: str,
    ) -> None:
        """Publish formalism-independent mission progress."""

        message = String()

        message.data = json.dumps(
            {
                'mission_id':
                    self.mission_id,

                'state_name':
                    state_name,

                'task_type':
                    task_type,

                'task_status':
                    status,

                'completed_navigation_tasks':
                    self.completed_navigation_tasks,
            }
        )

        self.progress_publisher.publish(
            message
        )

    def _finish_mission(
        self,
        result: str,
    ) -> None:
        """Stop FSM execution."""

        if self.mission_finished:
            return

        self.mission_finished = True

        self.timer.cancel()

        if result == 'SUCCESS':

            self.get_logger().info(
                'FSM mission completed successfully.'
            )
            return

        if result == 'SAFE_TERMINATED':

            self.get_logger().warning(
                'FSM mission terminated safely.'
            )

            if self.termination_reason:
                self.get_logger().warning(
                    f'Reason: '
                    f'{self.termination_reason}'
                )

            return


def main(args=None) -> None:

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = FSMExecutor()

        rclpy.spin(
            node
        )

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
