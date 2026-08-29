"""
Nominal ROS2 HTN mission executor.

This executor performs only normal mission execution.

Runtime recovery scenarios are intentionally excluded
from the nominal implementation so that scenario growth
can later be measured fairly.

Mission:
    YAML
      -> HTN planner
      -> primitive plan
      -> sequential execution
      -> /mission/progress
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
    Task,
    build_uav_domain,
)

from mission_formalism_evaluation.htn.htn_planner import (
    HTNPlanner,
)


class HTNExecutor(Node):
    """Execute a nominal UAV mission using HTN."""

    def __init__(
        self,
    ) -> None:

        super().__init__(
            "htn_executor"
        )

        # --------------------------------------------------------
        # Parameters
        # --------------------------------------------------------

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

        # --------------------------------------------------------
        # ROS interface
        # --------------------------------------------------------

        self.progress_publisher = (
            self.create_publisher(
                String,
                "/mission/progress",
                10,
            )
        )

        # --------------------------------------------------------
        # Mission
        # --------------------------------------------------------

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

        # --------------------------------------------------------
        # Execution state
        # --------------------------------------------------------

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
        }

        # --------------------------------------------------------
        # Planner
        # --------------------------------------------------------

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

        self.plan = (
            result.plan
        )

        # --------------------------------------------------------
        # Runtime bookkeeping
        # --------------------------------------------------------

        self.plan_index = 0

        self.active_task: (
            Task | None
        ) = None

        self.active_elapsed = 0.0

        self.completed_tasks = 0

        self.completed_navigation_tasks = 0

        self.mission_finished = False

        # --------------------------------------------------------
        # Logging
        # --------------------------------------------------------

        self.get_logger().info(
            "================================"
        )

        self.get_logger().info(
            "HTN Mission Executor"
        )

        self.get_logger().info(
            "Mode: NOMINAL"
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
            f"{len(self.plan)}"
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

            return (
                self.mission_file
            )

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

        mission = (
            data["mission"]
        )

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

        return (
            raw_id.upper()
        )

    # ============================================================
    # Execution
    # ============================================================

    def _timer_callback(
        self,
    ) -> None:

        if self.mission_finished:

            return

        if (
            self.plan_index
            >= len(self.plan)
        ):

            self._finish_mission()

            return

        if self.active_task is None:

            self.active_task = (
                copy.deepcopy(
                    self.plan[
                        self.plan_index
                    ]
                )
            )

            self.active_elapsed = 0.0

            self.get_logger().info(
                "HTN PRIMITIVE RUNNING: "
                f"{self._task_label(self.active_task)}"
            )

        self._publish_progress(
            state_name=(
                self._state_name(
                    self.active_task
                )
            ),
            task_type=(
                self._task_type(
                    self.active_task
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

        task = self.active_task

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
                    task
                )
            ),
            task_type=(
                self._task_type(
                    task
                )
            ),
            task_status="COMPLETED",
        )

        self.get_logger().info(
            "HTN PRIMITIVE SUCCESS: "
            f"{self._task_label(task)}"
        )

        self.completed_tasks += 1

        if (
            task[0]
            == "navigate"
        ):

            self.completed_navigation_tasks += 1

        self.plan_index += 1

        self.active_task = None

        self.active_elapsed = 0.0

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
    # Helpers
    # ============================================================

    def _task_type(
        self,
        task: Task,
    ) -> str:

        if task[0] in {
            "takeoff",
            "navigate",
            "land",
        }:

            return (
                task[0]
            )

        return str(
            task[1].get(
                "type",
                "generic",
            )
        ).lower()

    def _state_name(
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
            or ""
        )

        if source:

            return (
                "HTN_"
                f"{task[0].upper()}_"
                f"{source}"
            )

        return (
            "HTN_"
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

        return (
            task[0]
        )

    # ============================================================
    # Completion
    # ============================================================

    def _finish_mission(
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


def main(
    args=None,
) -> None:

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
