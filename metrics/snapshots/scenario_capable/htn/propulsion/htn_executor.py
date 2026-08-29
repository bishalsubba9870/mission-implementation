"""
GPS- and propulsion-capable ROS2 HTN mission executor.

The executor runs the primitive mission plan, updates
world-state facts, and invokes HTN recovery planning.
"""

from __future__ import annotations

import copy
import json
import os
import re

from typing import Any

import rclpy
import yaml

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node
from std_msgs.msg import String

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

        # ========================================================
        # Parameters
        # ========================================================

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

        # ========================================================
        # ROS interfaces
        # ========================================================

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

        # ========================================================
        # Mission
        # ========================================================

        self.mission_id = ""
        self.mission_name = ""

        self.tasks: list[
            dict[str, Any]
        ] = []

        mission_path = (
            self._resolve_mission_file()
        )

        self._load_mission(
            mission_path
        )

        # ========================================================
        # Runtime state
        # ========================================================

        self.world_state: dict[
            str,
            Any,
        ] = {
            "on_ground": True,
            "airborne": False,
            "location": "HOME",

            "gps_available": True,
            "gps_was_lost": False,
            "gps_recovery_failed": False,

            "propulsion_failure": False,

            "mission_interrupted": False,
            "safe_terminated": False,
        }

        # ========================================================
        # HTN planning
        # ========================================================

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

        # ========================================================
        # Execution bookkeeping
        # ========================================================

        self.main_index = 0

        self.active_task: (
            Task | None
        ) = None

        self.active_elapsed = 0.0

        self.paused_task: (
            Task | None
        ) = None

        self.paused_elapsed = 0.0

        self.recovery_plan: list[
            Task
        ] = []

        self.recovery_index = 0
        self.executing_recovery = False

        self.completed_tasks = 0
        self.completed_navigation_tasks = 0

        self.mission_finished = False

        # ========================================================
        # Startup information
        # ========================================================

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
            "Model: Totally Ordered "
            "State-Based HTN"
        )

        self.get_logger().info(
            f"Mission ID: "
            f"{self.mission_id}"
        )

        self.get_logger().info(
            f"Mission Name: "
            f"{self.mission_name}"
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

        method_count = sum(
            len(methods)
            for methods
            in self.domain.methods.values()
        )

        self.get_logger().info(
            f"HTN Methods: "
            f"{method_count}"
        )

        self.get_logger().info(
            "Mission Method Applications: "
            f"{len(result.trace.methods)}"
        )

        self.get_logger().info(
            "Generated Primitive Plan: "
            f"{len(self.main_plan)}"
        )

        self.get_logger().info(
            "================================"
        )

        self.timer = (
            self.create_timer(
                self.tick_period,
                self._timer_callback,
            )
        )

    # ============================================================
    # Mission loading
    # ============================================================

    def _resolve_mission_file(
        self,
    ) -> str:

        if os.path.isabs(
            self.mission_file
        ):

            return self.mission_file

        package_share = (
            get_package_share_directory(
                "mission_formalism_evaluation"
            )
        )

        return os.path.join(
            package_share,
            "missions",
            self.mission_file,
        )

    def _load_mission(
        self,
        mission_path: str,
    ) -> None:

        self.get_logger().info(
            f"Loading mission: "
            f"{mission_path}"
        )

        if not os.path.isfile(
            mission_path
        ):

            raise FileNotFoundError(
                mission_path
            )

        with open(
            mission_path,
            "r",
            encoding="utf-8",
        ) as file:

            data = yaml.safe_load(
                file
            )

        if not isinstance(
            data,
            dict,
        ):

            raise ValueError(
                "Mission YAML root "
                "must be a mapping."
            )

        if (
            "mission" not in data
            or "tasks" not in data
        ):

            raise ValueError(
                "Mission YAML requires "
                "'mission' and 'tasks'."
            )

        mission = data[
            "mission"
        ]

        raw_id = str(
            mission.get(
                "id",
                self.mission_file,
            )
        )

        self.mission_id = (
            self._normalise_mission_id(
                raw_id
            )
        )

        self.mission_name = str(
            mission.get(
                "name",
                raw_id,
            )
        )

        if not isinstance(
            data["tasks"],
            list,
        ):

            raise ValueError(
                "Mission tasks "
                "must be a list."
            )

        self.tasks = [
            copy.deepcopy(
                task
            )
            for task
            in data["tasks"]
            if isinstance(
                task,
                dict,
            )
        ]

    def _normalise_mission_id(
        self,
        raw_id: str,
    ) -> str:

        match = re.search(
            r"(\d+)",
            raw_id,
        )

        if match:

            return (
                "M"
                + match.group(1)
            )

        return raw_id.upper()

    # ============================================================
    # Runtime events
    # ============================================================

    def _runtime_event_callback(
        self,
        message: String,
    ) -> None:

        if self.mission_finished:

            return

        event = (
            self._extract_event(
                message.data
            )
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
            f"RUNTIME EVENT RECEIVED: "
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

    def _extract_event(
        self,
        raw_data: str,
    ) -> str:

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
                        "event",
                        "",
                    )
                ).strip().upper()

        except json.JSONDecodeError:

            pass

        return (
            raw_data
            .strip()
            .upper()
        )

    # ============================================================
    # Recovery planning
    # ============================================================

    def _plan_recovery(
        self,
        event: str,
    ) -> None:

        result = (
            self.planner.plan(
                self.world_state,
                [
                    (
                        "handle_runtime_condition",
                        {
                            "event": event,
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

    # ============================================================
    # Execution
    # ============================================================

    def _timer_callback(
        self,
    ) -> None:

        if self.mission_finished:

            return

        if self.executing_recovery:

            self._tick_recovery()

            return

        self._tick_mission()

    def _tick_mission(
        self,
    ) -> None:

        if (
            self.main_index
            >= len(self.main_plan)
        ):

            self._finish_success()

            return

        if self.active_task is None:

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

        self._execute_active(
            recovery=False
        )

    def _tick_recovery(
        self,
    ) -> None:

        if (
            self.recovery_index
            >= len(self.recovery_plan)
        ):

            self._complete_recovery()

            return

        if self.active_task is None:

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

    def _execute_active(
        self,
        recovery: bool,
    ) -> None:

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

    # ============================================================
    # Recovery completion
    # ============================================================

    def _complete_recovery(
        self,
    ) -> None:

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

    # ============================================================
    # Helpers
    # ============================================================

    def _is_navigating(
        self,
    ) -> bool:

        return (
            not self.executing_recovery
            and self.active_task
            is not None
            and self.active_task[0]
            == "navigate"
        )

    def _task_type(
        self,
        task: Task,
        recovery: bool,
    ) -> str:

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

    def _state_name(
        self,
        task: Task,
        recovery: bool,
    ) -> str:

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

    def _task_label(
        self,
        task: Task,
    ) -> str:

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

    # ============================================================
    # Progress
    # ============================================================

    def _publish_progress(
        self,
        state_name: str,
        task_type: str,
        task_status: str,
    ) -> None:

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

    # ============================================================
    # Completion
    # ============================================================

    def _finish_success(
        self,
    ) -> None:

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
            "================================"
        )

        self.get_logger().info(
            "MISSION COMPLETED"
        )

        self.get_logger().info(
            f"Completed tasks: "
            f"{self.completed_tasks}"
        )

        self.get_logger().info(
            "Completed navigation tasks: "
            f"{self.completed_navigation_tasks}"
        )

        self.get_logger().info(
            "================================"
        )

    def _finish_safe(
        self,
        reason: str,
    ) -> None:

        if self.mission_finished:

            return

        self.mission_finished = True
        self.timer.cancel()

        self.world_state[
            "safe_terminated"
        ] = True

        self._publish_progress(
            state_name="SAFE_TERMINATED",
            task_type="mission",
            task_status="SAFE_TERMINATED",
        )

        self.get_logger().warning(
            "================================"
        )

        self.get_logger().warning(
            "MISSION SAFE TERMINATED"
        )

        self.get_logger().warning(
            f"Reason: {reason}"
        )

        self.get_logger().warning(
            "================================"
        )


def main(
    args=None,
) -> None:

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = HTNExecutor()

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
