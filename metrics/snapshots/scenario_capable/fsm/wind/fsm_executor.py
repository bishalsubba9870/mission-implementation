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

    # =============================================================
    # Mission loading
    # =============================================================

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

    # =============================================================
    # FSM construction
    # =============================================================

    def _build_fsm(self) -> None:
        """
        Build nominal mission and runtime-condition states.

        Supported runtime events:
        - GPS_LOST
        - PROPULSION_FAILURE
        - COMMUNICATION_LOST
        - WIND_UNSAFE
        - BATTERY_CRITICAL
        - ABORT_REQUESTED
        """

        self.states = {
            'IDLE': None,

            'COMPLETED': None,

            'GPS_RECOVERY': {
                'type': 'recovery',
                'action': 'WAIT_FOR_GPS_STATUS',
            },

            'COMM_RECOVERY': {
                'type': 'recovery',
                'action': 'WAIT_FOR_COMMUNICATION',
            },

            'RTL': {
                'type': 'recovery',
                'action': 'RETURN_TO_HOME',
            },

            'LAND': {
                'type': 'recovery',
                'action': 'LAND',
            },

            # Wind-triggered mission abort.
            # Kept separate from operator ABORT_REQUESTED.
            'WIND_ABORT': {
                'type': 'recovery',
                'action': 'ABORT_MISSION',
            },

            'ABORT_MISSION': {
                'type': 'recovery',
                'action': 'ABORT_MISSION',
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

        # ---------------------------------------------------------
        # Initial transition
        # ---------------------------------------------------------

        self.transitions[
            (
                'IDLE',
                'START',
            )
        ] = task_state_names[0]

        # ---------------------------------------------------------
        # Nominal and runtime transitions
        # ---------------------------------------------------------

        for index, state_name in enumerate(
            task_state_names
        ):
            if (
                index
                == len(task_state_names) - 1
            ):
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
                # GPS loss
                self.transitions[
                    (
                        state_name,
                        'GPS_LOST',
                    )
                ] = 'GPS_RECOVERY'

                # Propulsion failure
                self.transitions[
                    (
                        state_name,
                        'PROPULSION_FAILURE',
                    )
                ] = 'LAND'

                # Communication loss
                self.transitions[
                    (
                        state_name,
                        'COMMUNICATION_LOST',
                    )
                ] = 'COMM_RECOVERY'

                # Unsafe wind:
                # abort nominal mission, then perform safe landing.
                self.transitions[
                    (
                        state_name,
                        'WIND_UNSAFE',
                    )
                ] = 'WIND_ABORT'

                # Battery
                self.transitions[
                    (
                        state_name,
                        'BATTERY_CRITICAL',
                    )
                ] = 'LAND'

                # Mission-level operator abort request
                self.transitions[
                    (
                        state_name,
                        'ABORT_REQUESTED',
                    )
                ] = 'ABORT_MISSION'

        # ---------------------------------------------------------
        # GPS recovery failure
        # ---------------------------------------------------------

        self.transitions[
            (
                'GPS_RECOVERY',
                'GPS_RECOVERY_FAILED',
            )
        ] = 'LAND'

        # ---------------------------------------------------------
        # Communication recovery failure
        # ---------------------------------------------------------

        self.transitions[
            (
                'COMM_RECOVERY',
                'COMMUNICATION_RECOVERY_FAILED',
            )
        ] = 'RTL'

        # ---------------------------------------------------------
        # Wind-triggered abort:
        #
        # WIND_ABORT -> LAND
        # ---------------------------------------------------------

        self.transitions[
            (
                'WIND_ABORT',
                'WIND_ABORT_COMPLETED',
            )
        ] = 'LAND'

        # ---------------------------------------------------------
        # Operator abort request:
        #
        # ABORT_MISSION -> RTL
        # ---------------------------------------------------------

        self.transitions[
            (
                'ABORT_MISSION',
                'ABORT_COMPLETED',
            )
        ] = 'RTL'

        # ---------------------------------------------------------
        # Generic RTL default:
        #
        # Communication recovery failure uses
        # RTL -> LAND.
        #
        # ABORT_REQUESTED is handled specially in the timer
        # callback so its RTL completion ends the experiment
        # at SAFE_TERMINATED.
        # ---------------------------------------------------------

        self.transitions[
            (
                'RTL',
                'RTL_COMPLETED',
            )
        ] = 'LAND'

        # ---------------------------------------------------------
        # Landing completion
        # ---------------------------------------------------------

        self.transitions[
            (
                'LAND',
                'LAND_COMPLETED',
            )
        ] = 'SAFE_TERMINATED'

    # =============================================================
    # State naming
    # =============================================================

    def _create_state_name(
        self,
        task: dict[str, Any],
        index: int,
    ) -> str:
        """Create unique FSM state name."""

        task_type = str(
            task.get(
                'type',
                '',
            )
        ).lower()

        if task_type == 'takeoff':

            return (
                f'TAKEOFF_{index}'
            )

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

            return (
                f'LAND_{index}'
            )

        raise ValueError(
            f'Unsupported task type '
            f'at index {index}: '
            f'{task_type}'
        )

    # =============================================================
    # Transition handling
    # =============================================================

    def _handle_event(
        self,
        event: str,
    ) -> None:
        """Perform FSM transition associated with event."""

        transition_key = (
            self.current_state,
            event,
        )

        next_state = (
            self.transitions.get(
                transition_key
            )
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

    # =============================================================
    # State entry
    # =============================================================

    def _enter_state(
        self,
        state_name: str,
    ) -> None:
        """Execute state-entry action."""

        if state_name == 'COMPLETED':

            self.get_logger().info(
                'STATE: COMPLETED'
            )

            self._finish_mission(
                result='SUCCESS'
            )

            return

        if (
            state_name
            == 'SAFE_TERMINATED'
        ):

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

    # =============================================================
    # Timer callback
    # =============================================================

    def _timer_callback(
        self,
    ) -> None:
        """Simulate completion of active FSM state."""

        if self.mission_finished:
            return

        if self.current_state in {
            'IDLE',
            'COMPLETED',
            'SAFE_TERMINATED',
        }:
            return

        # Wait for explicit recovery result.
        if self.current_state in {
            'GPS_RECOVERY',
            'COMM_RECOVERY',
        }:
            return

        state_data = (
            self.states.get(
                self.current_state
            )
        )

        if state_data is None:
            return

        state_type = str(
            state_data.get(
                'type',
                '',
            )
        ).lower()

        # ---------------------------------------------------------
        # Wind-triggered abort completed
        # ---------------------------------------------------------

        if (
            self.current_state
            == 'WIND_ABORT'
        ):

            self._publish_progress(
                state_name='WIND_ABORT',
                task_type='recovery',
                status='COMPLETED',
            )

            self.get_logger().info(
                'EVENT: WIND_ABORT_COMPLETED '
                'in WIND_ABORT'
            )

            self._handle_event(
                'WIND_ABORT_COMPLETED'
            )

            return

        # ---------------------------------------------------------
        # Operator abort completed
        # ---------------------------------------------------------

        if (
            self.current_state
            == 'ABORT_MISSION'
        ):

            self._publish_progress(
                state_name='ABORT_MISSION',
                task_type='recovery',
                status='COMPLETED',
            )

            self.get_logger().info(
                'EVENT: ABORT_COMPLETED '
                'in ABORT_MISSION'
            )

            self._handle_event(
                'ABORT_COMPLETED'
            )

            return

        # ---------------------------------------------------------
        # Return-to-home completed
        # ---------------------------------------------------------

        if self.current_state == 'RTL':

            self._publish_progress(
                state_name='RTL',
                task_type='recovery',
                status='COMPLETED',
            )

            self.get_logger().info(
                'EVENT: RTL_COMPLETED in RTL'
            )

            # ABORT_REQUESTED expected outcome:
            #
            # ABORT_MISSION -> RETURN_TO_HOME
            #
            # Once home is reached, the experimental
            # requirement is satisfied.
            if (
                self.termination_reason
                == 'ABORT_REQUESTED'
            ):

                previous_state = (
                    self.current_state
                )

                self.current_state = (
                    'SAFE_TERMINATED'
                )

                self.get_logger().info(
                    f'TRANSITION: '
                    f'{previous_state} '
                    f'--[RTL_COMPLETED]--> '
                    f'SAFE_TERMINATED'
                )

                self._enter_state(
                    'SAFE_TERMINATED'
                )

                return

            # Communication-failure path retains:
            #
            # RTL -> LAND
            self._handle_event(
                'RTL_COMPLETED'
            )

            return

        # ---------------------------------------------------------
        # Landing completed
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # Nominal task completion
        # ---------------------------------------------------------

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

    # =============================================================
    # Runtime event callback
    # =============================================================

    def _runtime_event_callback(
        self,
        message: String,
    ) -> None:
        """Receive runtime event from scenario injector."""

        if self.mission_finished:
            return

        event = (
            self._extract_runtime_event(
                message.data
            )
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

            if not self._is_navigating():

                self.get_logger().warning(
                    'GPS_LOST ignored because '
                    'FSM is not navigating.'
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

            self._resume_interrupted_state(
                event='GPS_AVAILABLE',
                message=(
                    'GPS recovered. '
                    'Resuming interrupted mission task.'
                ),
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

        if (
            event
            == 'PROPULSION_FAILURE'
        ):

            if not self._is_navigating():

                self.get_logger().warning(
                    'PROPULSION_FAILURE ignored because '
                    'FSM is not navigating.'
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

        # =========================================================
        # COMMUNICATION LOST
        # =========================================================

        if (
            event
            == 'COMMUNICATION_LOST'
        ):

            if not self._is_navigating():

                self.get_logger().warning(
                    'COMMUNICATION_LOST ignored because '
                    'FSM is not navigating.'
                )

                return

            self.interrupted_state = (
                self.current_state
            )

            self._handle_event(
                'COMMUNICATION_LOST'
            )

            return

        # =========================================================
        # COMMUNICATION RESTORED
        # =========================================================

        if (
            event
            == 'COMMUNICATION_RESTORED'
        ):

            if (
                self.current_state
                != 'COMM_RECOVERY'
            ):

                self.get_logger().warning(
                    'COMMUNICATION_RESTORED received '
                    'outside COMM_RECOVERY.'
                )

                return

            self._resume_interrupted_state(
                event='COMMUNICATION_RESTORED',
                message=(
                    'Communication restored. '
                    'Resuming interrupted mission task.'
                ),
            )

            return

        # =========================================================
        # COMMUNICATION RECOVERY FAILED
        # =========================================================

        if (
            event
            == 'COMMUNICATION_RECOVERY_FAILED'
        ):

            if (
                self.current_state
                != 'COMM_RECOVERY'
            ):

                self.get_logger().warning(
                    'COMMUNICATION_RECOVERY_FAILED '
                    'received outside COMM_RECOVERY.'
                )

                return

            self.termination_reason = (
                'COMMUNICATION_RECOVERY_FAILED'
            )

            self.interrupted_state = None

            self._handle_event(
                'COMMUNICATION_RECOVERY_FAILED'
            )

            return

        # =========================================================
        # WIND UNSAFE
        # =========================================================

        if event == 'WIND_UNSAFE':

            if not self._is_navigating():

                self.get_logger().warning(
                    'WIND_UNSAFE ignored because '
                    'FSM is not navigating.'
                )

                return

            self.termination_reason = (
                'WIND_UNSAFE'
            )

            self.interrupted_state = None

            self._handle_event(
                'WIND_UNSAFE'
            )

            return

        # =========================================================
        # BATTERY
        # =========================================================

        if event == 'BATTERY_CRITICAL':

            if not self._is_navigating():

                self.get_logger().warning(
                    'BATTERY_CRITICAL ignored because '
                    'FSM is not navigating.'
                )

                return

            self.termination_reason = (
                'BATTERY_CRITICAL'
            )

            self.interrupted_state = None

            self._handle_event(
                'BATTERY_CRITICAL'
            )

            return

        # =========================================================
        # ABORT REQUESTED
        # =========================================================

        if event == 'ABORT_REQUESTED':

            if not self._is_navigating():

                self.get_logger().warning(
                    'ABORT_REQUESTED ignored because '
                    'FSM is not navigating.'
                )

                return

            self.termination_reason = (
                'ABORT_REQUESTED'
            )

            self.interrupted_state = None

            self._handle_event(
                'ABORT_REQUESTED'
            )

            return

        # =========================================================
        # Unsupported event
        # =========================================================

        self.get_logger().warning(
            f'Runtime event not yet supported '
            f'by FSM: {event}'
        )

    # =============================================================
    # Helpers
    # =============================================================

    def _is_navigating(
        self,
    ) -> bool:
        """Check whether current FSM state is navigation."""

        state_data = self.states.get(
            self.current_state
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

    def _resume_interrupted_state(
        self,
        event: str,
        message: str,
    ) -> None:
        """Resume interrupted navigation state."""

        if (
            self.interrupted_state
            is None
        ):

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
            f'--[{event}]--> '
            f'{resume_state}'
        )

        self.get_logger().info(
            message
        )

        self._enter_state(
            resume_state
        )

    # =============================================================
    # Event parser
    # =============================================================

    def _extract_runtime_event(
        self,
        raw_data: str,
    ) -> str:
        """Parse JSON or plain-string runtime event."""

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

    # =============================================================
    # Progress publication
    # =============================================================

    def _publish_progress(
        self,
        state_name: str,
        task_type: str,
        status: str,
    ) -> None:
        """Publish mission progress."""

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

    # =============================================================
    # Mission completion
    # =============================================================

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
