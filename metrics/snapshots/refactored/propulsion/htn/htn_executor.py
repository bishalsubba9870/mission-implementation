"""GPS- and propulsion-capable ROS2 HTN mission executor."""

from __future__ import annotations

import copy
import json

from typing import Any

import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from mission_formalism_evaluation.common.mission_loader import (
    load_mission,
)
from mission_formalism_evaluation.common.runtime_condition import (
    parse_runtime_event,
)
from mission_formalism_evaluation.htn.htn_domain import (
    RUNTIME_EVENT_UPDATES,
    Task,
    build_uav_domain,
)
from mission_formalism_evaluation.htn.htn_planner import (
    HTNPlanner,
)


class HTNExecutor(Node):
    """Execute a GPS- and propulsion-capable UAV mission."""

    def __init__(
        self,
    ) -> None:

        super().__init__(
            "htn_executor"
        )

        self.declare_parameter(
            "mission_file",
            "mission_1.yaml",
        )

        self.declare_parameter(
            "state_duration",
            0.2,
        )

        self.mission_file = str(
            self.get_parameter(
                "mission_file"
            ).value
        )

        self.state_duration = float(
            self.get_parameter(
                "state_duration"
            ).value
        )

        self.tick_period = 0.02

        self.progress_publisher = (
            self.create_publisher(
                String,
                "/mission/progress",
                10,
            )
        )

        self.event_subscription = (
            self.create_subscription(
                String,
                "/mission/event",
                self._runtime_event_callback,
                10,
            )
        )

        (
            self.mission_id,
            self.mission_name,
            raw_tasks,
            mission_path,
        ) = load_mission(
            "mission_formalism_evaluation",
            self.mission_file,
        )

        self.tasks: list[
            dict[str, Any]
        ] = [
            copy.deepcopy(
                task
            )
            for task
            in raw_tasks
            if isinstance(
                task,
                dict,
            )
        ]

        if not self.tasks:

            raise ValueError(
                "Mission has zero valid tasks."
            )

        self.get_logger().info(
            f"Loading mission: "
            f"{mission_path}"
        )

        self.world_state: dict[
            str,
            Any,
        ] = {
            "on_ground":
                True,

            "airborne":
                False,

            "location":
                "HOME",

            "gps_available":
                True,

            "gps_was_lost":
                False,

            "gps_recovery_failed":
                False,

            "propulsion_failure":
                False,

            "mission_interrupted":
                False,

            "safe_terminated":
                False,
        }

        self.domain = (
            build_uav_domain()
        )

        self.planner = (
            HTNPlanner(
                self.domain
            )
        )

        task_network: list[
            Task
        ] = [
            (
                "execute_mission",
                {
                    "mission_tasks":
                        self.tasks,
                },
            )
        ]

        result = (
            self.planner.plan(
                self.world_state,
                task_network,
            )
        )

        if not result.success:

            raise RuntimeError(
                "Initial HTN planning "
                f"failed: {result.reason}"
            )

        self.main_plan = result.plan
        self.main_index = 0

        self.recovery_plan: list[
            Task
        ] = []

        self.recovery_index = 0
        self.executing_recovery = False

        self.active_task: (
            Task | None
        ) = None

        self.active_elapsed = 0.0

        self.paused_task: (
            Task | None
        ) = None

        self.paused_elapsed = 0.0

        self.completed_tasks = 0
        self.completed_navigation_tasks = 0

        self.mission_finished = False
        self.termination_reason = None

        self._log_startup(
            result
        )

        self.timer = (
            self.create_timer(
                self.tick_period,
                self._timer_callback,
            )
        )

    def _runtime_event_callback(
        self,
        message: String,
    ) -> None:
        """Process supported runtime events."""

        if self.mission_finished:

            return

        event = parse_runtime_event(
            message.data
        )

        updates = (
            RUNTIME_EVENT_UPDATES.get(
                event
            )
        )

        if updates is None:

            self.get_logger().warning(
                "Runtime event not supported "
                f"by current HTN: {event}"
            )

            return

        if (
            event
            in {
                "GPS_LOST",
                "PROPULSION_FAILURE",
            }
            and not self._is_navigating()
        ):

            self.get_logger().warning(
                f"{event} ignored because "
                "HTN is not navigating."
            )

            return

        self.world_state.update(
            updates
        )

        self.get_logger().warning(
            "RUNTIME EVENT RECEIVED: "
            f"{event}"
        )

        if event in {
            "GPS_LOST",
            "PROPULSION_FAILURE",
        }:

            self._pause_mission_task()

        self._plan_recovery(
            event
        )

    def _plan_recovery(
        self,
        event: str,
    ) -> None:
        """Generate recovery plan from current world state."""

        result = (
            self.planner.plan(
                self.world_state,
                [
                    (
                        "handle_runtime_condition",
                        {
                            "event":
                                event,
                        },
                    )
                ],
            )
        )

        if not result.success:

            self.get_logger().error(
                "HTN recovery planning "
                f"failed for {event}."
            )

            return

        self.recovery_plan = (
            result.plan
        )

        self.recovery_index = 0
        self.executing_recovery = True

        self.active_task = None
        self.active_elapsed = 0.0

        self.get_logger().warning(
            "HTN REPLANNING"
        )

        self.get_logger().warning(
            "Selected method: "
            + " -> ".join(
                result.trace.methods
            )
        )

        self.get_logger().warning(
            "Recovery plan: "
            + " -> ".join(
                task[0]
                for task
                in self.recovery_plan
            )
        )

    def _pause_mission_task(
        self,
    ) -> None:
        """Pause the current mission primitive."""

        if self.active_task is None:

            return

        self.paused_task = (
            copy.deepcopy(
                self.active_task
            )
        )

        self.paused_elapsed = (
            self.active_elapsed
        )

        self.active_task = None
        self.active_elapsed = 0.0

        self.get_logger().warning(
            "HTN PAUSED: "
            f"{self._task_label(self.paused_task)}"
        )

    def _timer_callback(
        self,
    ) -> None:
        """Advance mission or recovery execution."""

        if self.mission_finished:

            return

        if self.executing_recovery:

            self._tick_recovery()

            return

        self._tick_mission()

    def _tick_mission(
        self,
    ) -> None:
        """Advance the main primitive plan."""

        if (
            self.main_index
            >= len(self.main_plan)
        ):

            self._finish_success()

            return

        if self.active_task is None:

            self._start_main_task()

        self._execute_active(
            recovery=False
        )

    def _start_main_task(
        self,
    ) -> None:
        """Start the current mission primitive."""

        self.active_task = (
            copy.deepcopy(
                self.main_plan[
                    self.main_index
                ]
            )
        )

        self.active_elapsed = 0.0

        self.get_logger().info(
            "HTN PRIMITIVE RUNNING: "
            f"{self._task_label(self.active_task)}"
        )

    def _tick_recovery(
        self,
    ) -> None:
        """Advance the active recovery plan."""

        if (
            self.recovery_index
            >= len(self.recovery_plan)
        ):

            self._complete_recovery()

            return

        if self.active_task is None:

            self._start_recovery_task()

        if (
            self.active_task[0]
            == "wait_for_gps"
        ):

            self._publish_progress(
                state_name="HTN_GPS_RECOVERY",
                task_type="recovery",
                task_status="RUNNING",
            )

            return

        self._execute_active(
            recovery=True
        )

    def _start_recovery_task(
        self,
    ) -> None:
        """Start the current recovery primitive."""

        self.active_task = (
            copy.deepcopy(
                self.recovery_plan[
                    self.recovery_index
                ]
            )
        )

        self.active_elapsed = 0.0

        self.get_logger().info(
            "HTN RECOVERY RUNNING: "
            f"{self._task_label(self.active_task)}"
        )

    def _execute_active(
        self,
        recovery: bool,
    ) -> None:
        """Execute the active primitive task."""

        if self.active_task is None:

            return

        task = self.active_task

        self._publish_progress(
            state_name=(
                self._state_name(
                    task,
                    recovery,
                )
            ),
            task_type=(
                self._task_type(
                    task,
                    recovery,
                )
            ),
            task_status="RUNNING",
        )

        self.active_elapsed += (
            self.tick_period
        )

        if (
            self.active_elapsed
            < self.state_duration
        ):

            return

        operator = (
            self.domain
            .operators[
                task[0]
            ]
        )

        operator.effect(
            self.world_state,
            task,
        )

        self._publish_progress(
            state_name=(
                self._state_name(
                    task,
                    recovery,
                )
            ),
            task_type=(
                self._task_type(
                    task,
                    recovery,
                )
            ),
            task_status="COMPLETED",
        )

        self.get_logger().info(
            "HTN PRIMITIVE SUCCESS: "
            f"{self._task_label(task)}"
        )

        if recovery:

            self.recovery_index += 1

        else:

            self.main_index += 1
            self.completed_tasks += 1

            if task[0] == "navigate":

                self.completed_navigation_tasks += 1

        if task[0] == "safe_terminate":

            reason = str(
                task[1].get(
                    "reason",
                    "SAFE_TERMINATED",
                )
            )

            self._finish_safe(
                reason
            )

            return

        self.active_task = None
        self.active_elapsed = 0.0

    def _complete_recovery(
        self,
    ) -> None:
        """Finish recovery and resume mission if possible."""

        self.executing_recovery = False

        self.recovery_plan = []
        self.recovery_index = 0

        self.active_task = None
        self.active_elapsed = 0.0

        if self.world_state[
            "safe_terminated"
        ]:

            return

        if self.world_state[
            "mission_interrupted"
        ]:

            return

        if self.paused_task is None:

            return

        self.active_task = (
            self.paused_task
        )

        self.active_elapsed = (
            self.paused_elapsed
        )

        self.paused_task = None
        self.paused_elapsed = 0.0

        self.get_logger().info(
            "HTN RESUMED: "
            f"{self._task_label(self.active_task)}"
        )

    def _is_navigating(
        self,
    ) -> bool:
        """Return whether HTN is executing navigation."""

        return (
            not self.executing_recovery
            and self.active_task
            is not None
            and self.active_task[0]
            == "navigate"
        )

    def _publish_progress(
        self,
        state_name: str,
        task_type: str,
        task_status: str,
    ) -> None:
        """Publish injector-compatible progress."""

        message = String()

        message.data = json.dumps(
            {
                "mission_id":
                    self.mission_id,

                "state_name":
                    state_name,

                "task_type":
                    task_type,

                "task_status":
                    task_status,

                "completed_navigation_tasks":
                    self.completed_navigation_tasks,
            }
        )

        self.progress_publisher.publish(
            message
        )

    @staticmethod
    def _task_type(
        task: Task,
        recovery: bool,
    ) -> str:
        """Return normalized primitive type."""

        if recovery:

            return "recovery"

        if task[0] in {
            "takeoff",
            "navigate",
            "land",
        }:

            return task[0]

        return str(
            task[1].get(
                "type",
                "generic",
            )
        ).lower()

    @staticmethod
    def _state_name(
        task: Task,
        recovery: bool,
    ) -> str:
        """Return progress state name."""

        prefix = (
            "HTN_RECOVERY"
            if recovery
            else "HTN"
        )

        source = (
            task[1].get(
                "id"
            )
            or task[1].get(
                "waypoint"
            )
            or ""
        )

        if source:

            return (
                f"{prefix}_"
                f"{task[0].upper()}_"
                f"{source}"
            )

        return (
            f"{prefix}_"
            f"{task[0].upper()}"
        )

    @staticmethod
    def _task_label(
        task: Task,
    ) -> str:
        """Return readable primitive-task label."""

        source = (
            task[1].get(
                "id"
            )
            or task[1].get(
                "waypoint"
            )
            or task[1].get(
                "name"
            )
        )

        if source:

            return (
                f"{task[0]} "
                f"[{source}]"
            )

        return task[0]

    def _log_startup(
        self,
        result,
    ) -> None:
        """Log GPS- and propulsion-capable HTN structure."""

        method_count = sum(
            len(methods)
            for methods
            in self.domain.methods.values()
        )

        self.get_logger().info(
            "================================"
        )

        self.get_logger().info(
            "HTN Mission Executor"
        )

        self.get_logger().info(
            "Mode: GPS_PROPULSION_CAPABLE"
        )

        self.get_logger().info(
            f"Mission ID: "
            f"{self.mission_id}"
        )

        self.get_logger().info(
            f"Mission Items: "
            f"{len(self.tasks)}"
        )

        self.get_logger().info(
            "HTN Primitive Operators: "
            f"{len(self.domain.operators)}"
        )

        self.get_logger().info(
            "HTN Compound Tasks: "
            f"{len(self.domain.compound_tasks)}"
        )

        self.get_logger().info(
            "HTN Methods: "
            f"{method_count}"
        )

        self.get_logger().info(
            "Generated Primitive Plan: "
            f"{len(self.main_plan)}"
        )

        self.get_logger().info(
            "================================"
        )

    def _finish_success(
        self,
    ) -> None:
        """Complete mission successfully."""

        if self.mission_finished:

            return

        self.mission_finished = True
        self.timer.cancel()

        self._publish_progress(
            state_name="COMPLETED",
            task_type="mission",
            task_status="COMPLETED",
        )

        self.get_logger().info(
            "MISSION COMPLETED"
        )

        self.get_logger().info(
            f"Completed tasks: "
            f"{self.completed_tasks}"
        )

    def _finish_safe(
        self,
        reason: str,
    ) -> None:
        """Safely terminate mission."""

        if self.mission_finished:

            return

        self.mission_finished = True
        self.termination_reason = reason

        self.timer.cancel()

        self._publish_progress(
            state_name="SAFE_TERMINATED",
            task_type="mission",
            task_status="COMPLETED",
        )

        self.get_logger().warning(
            "MISSION SAFE TERMINATED"
        )

        self.get_logger().warning(
            f"Reason: {reason}"
        )


def main(
    args=None,
) -> None:
    """Run HTN executor."""

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = (
            HTNExecutor()
        )

        rclpy.spin(
            node
        )

    except (
        FileNotFoundError,
        KeyError,
        RuntimeError,
        TypeError,
        ValueError,
    ) as error:

        print(
            "[HTN EXECUTOR ERROR] "
            f"{error}"
        )

    finally:

        if node is not None:

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
