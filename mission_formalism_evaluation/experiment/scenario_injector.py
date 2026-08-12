import json
import os
from typing import Any

import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class ScenarioInjector(Node):
    """Publish one configured runtime condition at a mission-progress trigger."""

    SUPPORTED_TEST_CASES = {
        'baseline',
        'gps_lost',
        'battery_critical',
        'communication_lost',
        'abort_requested',
        'propulsion_failure',
        'wind_unsafe',
    }

    def __init__(self) -> None:
        super().__init__('scenario_injector')

        self.declare_parameter('mission', 'M1')
        self.declare_parameter('test_case', 'baseline')
        self.declare_parameter(
            'scenario_file',
            'scenario_definitions.yaml',
        )
        self.declare_parameter(
            'trigger_file',
            'trigger_definitions.yaml',
        )
        self.declare_parameter(
            'trigger_override',
            -1,
        )

        self.mission_id = (
            self.get_parameter('mission')
            .get_parameter_value()
            .string_value
            .upper()
        )

        self.test_case = (
            self.get_parameter('test_case')
            .get_parameter_value()
            .string_value
            .lower()
        )

        scenario_filename = (
            self.get_parameter('scenario_file')
            .get_parameter_value()
            .string_value
        )

        trigger_filename = (
            self.get_parameter('trigger_file')
            .get_parameter_value()
            .string_value
        )

        self.trigger_override = (
            self.get_parameter('trigger_override')
            .get_parameter_value()
            .integer_value
        )

        self.event_name = 'NONE'
        self.expected_decision = ''
        self.inject_enabled = False

        self.trigger_type = ''
        self.trigger_value = 0
        self.required_task_type = ''
        self.required_task_status = ''

        self.event_injected = False

        package_share = get_package_share_directory(
            'mission_formalism_evaluation'
        )

        scenario_path = os.path.join(
            package_share,
            'scenarios',
            scenario_filename,
        )

        trigger_path = os.path.join(
            package_share,
            'scenarios',
            trigger_filename,
        )

        self._validate_parameters()
        self._load_scenario_definition(scenario_path)
        self._load_trigger_definition(trigger_path)

        self.event_publisher = self.create_publisher(
            String,
            '/mission/event',
            10,
        )

        self.progress_subscription = self.create_subscription(
            String,
            '/mission/progress',
            self._progress_callback,
            10,
        )

        self._print_configuration()

    def _validate_parameters(self) -> None:
        """Validate mission and test-case parameters."""

        valid_missions = {'M1', 'M2', 'M3', 'M4'}

        if self.mission_id not in valid_missions:
            raise ValueError(
                f'Unsupported mission: {self.mission_id}. '
                f'Choose one of {sorted(valid_missions)}.'
            )

        if self.test_case not in self.SUPPORTED_TEST_CASES:
            raise ValueError(
                f'Unsupported test case: {self.test_case}. '
                f'Choose one of '
                f'{sorted(self.SUPPORTED_TEST_CASES)}.'
            )

    def _load_yaml(self, path: str) -> dict[str, Any]:
        """Load a YAML file and return its root mapping."""

        if not os.path.exists(path):
            raise FileNotFoundError(
                f'Configuration file not found: {path}'
            )

        with open(path, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)

        if not isinstance(data, dict):
            raise ValueError(
                f'YAML root must be a mapping: {path}'
            )

        return data

    def _load_scenario_definition(
        self,
        scenario_path: str,
    ) -> None:
        """Load the selected scenario definition."""

        data = self._load_yaml(scenario_path)

        test_cases = data.get('test_cases')

        if not isinstance(test_cases, dict):
            raise ValueError(
                "scenario_definitions.yaml requires "
                "a 'test_cases' mapping."
            )

        selected = test_cases.get(self.test_case)

        if not isinstance(selected, dict):
            raise ValueError(
                f'No definition found for test case: '
                f'{self.test_case}'
            )

        self.inject_enabled = bool(
            selected.get('inject', False)
        )

        self.event_name = str(
            selected.get('event', 'NONE')
        ).upper()

        self.expected_decision = str(
            selected.get('expected_decision', '')
        ).upper()

    def _load_trigger_definition(
        self,
        trigger_path: str,
    ) -> None:
        """Load the trigger configuration for the selected mission."""

        data = self._load_yaml(trigger_path)

        mission_triggers = data.get('mission_triggers')

        if not isinstance(mission_triggers, dict):
            raise ValueError(
                "trigger_definitions.yaml requires "
                "a 'mission_triggers' mapping."
            )

        trigger = mission_triggers.get(self.mission_id)

        if not isinstance(trigger, dict):
            raise ValueError(
                f'No trigger configured for mission '
                f'{self.mission_id}.'
            )

        self.trigger_type = str(
            trigger.get('trigger_type', '')
        ).upper()

        self.trigger_value = int(
            trigger.get('trigger_value', 0)
        )

        self.required_task_type = str(
            trigger.get(
                'required_current_task_type',
                'navigate',
            )
        ).lower()

        self.required_task_status = str(
            trigger.get(
                'required_current_task_status',
                'RUNNING',
            )
        ).upper()

        if self.trigger_override >= 0:
            self.trigger_value = self.trigger_override

        if (
            self.trigger_type
            != 'COMPLETED_NAVIGATION_COUNT'
        ):
            raise ValueError(
                f'Unsupported trigger type: '
                f'{self.trigger_type}'
            )

    def _print_configuration(self) -> None:
        """Print the selected experiment settings."""

        self.get_logger().info(
            f'Mission: {self.mission_id}'
        )

        self.get_logger().info(
            f'Test case: {self.test_case}'
        )

        self.get_logger().info(
            f'Event: {self.event_name}'
        )

        self.get_logger().info(
            f'Expected decision: '
            f'{self.expected_decision}'
        )

        if not self.inject_enabled:
            self.get_logger().info(
                'Baseline selected: no event will be injected.'
            )
            return

        self.get_logger().info(
            f'Trigger: {self.trigger_type} '
            f'= {self.trigger_value}'
        )

        self.get_logger().info(
            f'Required current task: '
            f'{self.required_task_type} / '
            f'{self.required_task_status}'
        )

        self.get_logger().info(
            'Waiting for /mission/progress ...'
        )

    def _progress_callback(
        self,
        message: String,
    ) -> None:
        """Check mission progress and publish the selected event."""

        if not self.inject_enabled:
            return

        if self.event_injected:
            return

        try:
            progress = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().error(
                f'Invalid JSON on /mission/progress: '
                f'{message.data}'
            )
            return

        if not isinstance(progress, dict):
            self.get_logger().error(
                'Mission progress must be a JSON object.'
            )
            return

        if str(
            progress.get('mission_id', '')
        ).upper() != self.mission_id:
            return

        completed_navigation_tasks = int(
            progress.get(
                'completed_navigation_tasks',
                -1,
            )
        )

        task_type = str(
            progress.get('task_type', '')
        ).lower()

        task_status = str(
            progress.get('task_status', '')
        ).upper()

        trigger_matches = (
            completed_navigation_tasks
            == self.trigger_value
            and task_type
            == self.required_task_type
            and task_status
            == self.required_task_status
        )

        if not trigger_matches:
            return

        self._publish_event(progress)

    def _publish_event(
        self,
        progress: dict[str, Any],
    ) -> None:
        """Publish the selected runtime condition once."""

        event_message = String()

        event_message.data = json.dumps({
            'mission_id': self.mission_id,
            'test_case': self.test_case,
            'event': self.event_name,
            'expected_decision':
                self.expected_decision,
            'trigger_type': self.trigger_type,
            'trigger_value': self.trigger_value,
            'trigger_state':
                progress.get('state_name', ''),
            'completed_navigation_tasks':
                progress.get(
                    'completed_navigation_tasks',
                    -1,
                ),
        })

        self.event_publisher.publish(event_message)
        self.event_injected = True

        self.get_logger().warning(
            f'INJECTED EVENT: {self.event_name}'
        )

        self.get_logger().info(
            f"Trigger state: "
            f"{progress.get('state_name', 'UNKNOWN')}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = ScenarioInjector()
        rclpy.spin(node)

    except (
        FileNotFoundError,
        KeyError,
        TypeError,
        ValueError,
    ) as error:
        print(f'[SCENARIO INJECTOR ERROR] {error}')

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
