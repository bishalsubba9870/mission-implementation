"""
GPS-capable Expert System executor for UAV mission execution.
"""

from __future__ import annotations

import json
import os

import rclpy
import yaml

from ament_index_python.packages import (
    get_package_share_directory,
)
from rclpy.node import Node
from std_msgs.msg import String

from mission_formalism_evaluation.expert_system.es_domain import (
    RUNTIME_EVENT_UPDATES,
    build_gps_knowledge_base,
)
from mission_formalism_evaluation.expert_system.rule_engine import (
    RuleEngine,
    WorkingMemory,
)


class ExpertSystemExecutor(Node):
    """Execute missions using a GPS-capable production-rule system."""

    def __init__(
        self,
    ) -> None:

        super().__init__(
            "expert_system_executor"
        )

        self.declare_parameter(
            "mission_file",
            "mission_2.yaml",
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

        self.mission_id = ""
        self.mission_name = ""

        self.tasks: list[
            dict
        ] = []

        self.current_task_index = 0
        self.current_task = None
        self.current_task_running = False

        self.completed_tasks = 0
        self.completed_navigation_tasks = 0

        self.mission_finished = False

        self.recovery_active = False
        self.recovery_action = None
        self.termination_reason = ""

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

        mission_path = (
            self._resolve_mission_file(
                self.mission_file
            )
        )

        self._load_mission(
            mission_path
        )

        self.working_memory = WorkingMemory(
            {
                "current_task_type": None,
                "current_task_index": None,
                "all_tasks_processed": False,
                "mission_complete": False,
                "mission_interrupted": False,
                "decision": None,

                "gps_available": True,
                "gps_was_lost": False,
                "gps_recovery_failed": False,

                "safe_termination_ready": False,
                "safe_terminated": False,
                "termination_reason": "",
            }
        )

        self.knowledge_base = (
            build_gps_knowledge_base()
        )

        self.rule_engine = RuleEngine(
            rules=self.knowledge_base,
            working_memory=self.working_memory,
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Expert System Mission Executor"
        )

        self.get_logger().info(
            "Mode: GPS CAPABLE"
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            f"Mission ID: {self.mission_id}"
        )

        self.get_logger().info(
            f"Mission Name: {self.mission_name}"
        )

        self.get_logger().info(
            f"Mission Items: {len(self.tasks)}"
        )

        self.get_logger().info(
            f"Knowledge Base Rules: "
            f"{len(self.knowledge_base)}"
        )

        self.timer = self.create_timer(
            self.state_duration,
            self._timer_callback,
        )

    # ================================================================
    # Mission file resolution
    # ================================================================

    def _resolve_mission_file(
        self,
        mission_file: str,
    ) -> str:

        if os.path.isabs(
            mission_file
        ):

            if os.path.isfile(
                mission_file
            ):
                return mission_file

            raise FileNotFoundError(
                mission_file
            )

        share_dir = (
            get_package_share_directory(
                "mission_formalism_evaluation"
            )
        )

        candidates = [
            os.path.join(
                share_dir,
                "missions",
                mission_file,
            ),

            os.path.join(
                share_dir,
                mission_file,
            ),
        ]

        for path in candidates:

            if os.path.isfile(
                path
            ):
                return path

        raise FileNotFoundError(
            mission_file
        )

    # ================================================================
    # Mission loading
    # ================================================================

    def _load_mission(
        self,
        mission_path: str,
    ) -> None:

        self.get_logger().info(
            f"Loading mission: {mission_path}"
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
                "Mission YAML root must be a mapping."
            )

        if (
            "mission" not in data
            or "tasks" not in data
        ):
            raise ValueError(
                "Mission YAML requires "
                "'mission' and 'tasks'."
            )

        mission_data = data[
            "mission"
        ]

        tasks = data[
            "tasks"
        ]

        if not isinstance(
            mission_data,
            dict,
        ):
            raise ValueError(
                "'mission' must be a mapping."
            )

        if (
            not isinstance(
                tasks,
                list,
            )
            or not tasks
        ):
            raise ValueError(
                "Mission must contain "
                "at least one task."
            )

        self.mission_id = str(
            mission_data[
                "id"
            ]
        )

        self.mission_name = str(
            mission_data[
                "name"
            ]
        )

        self.tasks = tasks

    # ================================================================
    # Inference execution
    # ================================================================

    def _timer_callback(
        self,
    ) -> None:

        if self.mission_finished:
            return

        if self.recovery_active:

            self._tick_recovery()
            return

        if self.current_task_running:

            self._complete_current_task()
            return

        if (
            self.current_task_index
            >= len(
                self.tasks
            )
        ):

            self._complete_mission()
            return

        self._start_current_task()

    def _start_current_task(
        self,
    ) -> None:

        self.current_task = self.tasks[
            self.current_task_index
        ]

        task_type = self._task_type(
            self.current_task
        )

        self.rule_engine.update_facts(
            {
                "current_task_type":
                    task_type,

                "current_task_index":
                    self.current_task_index,

                "all_tasks_processed":
                    False,

                "decision":
                    None,
            }
        )

        result = (
            self.rule_engine.inference_cycle()
        )

        if result.selected_rule is None:

            raise RuntimeError(
                f"No production rule matched "
                f"task {self.current_task_index}: "
                f"{task_type}"
            )

        self._log_inference(
            result
        )

        state_name = (
            self._task_label(
                self.current_task,
                self.current_task_index,
            )
        )

        self.get_logger().info(
            f"ES RUNNING: "
            f"{state_name}"
        )

        self.current_task_running = True

        self._publish_progress(
            state_name=state_name,
            task_type=task_type,
            status="RUNNING",
        )

    def _complete_current_task(
        self,
    ) -> None:

        if self.current_task is None:
            return

        task_type = self._task_type(
            self.current_task
        )

        state_name = (
            self._task_label(
                self.current_task,
                self.current_task_index,
            )
        )

        self._publish_progress(
            state_name=state_name,
            task_type=task_type,
            status="COMPLETED",
        )

        self.get_logger().info(
            f"ES COMPLETED: "
            f"{state_name}"
        )

        if task_type == "navigate":

            self.completed_navigation_tasks += 1

        self.completed_tasks += 1
        self.current_task_index += 1

        self.current_task = None
        self.current_task_running = False

        self.rule_engine.update_facts(
            {
                "current_task_type":
                    None,

                "current_task_index":
                    None,

                "decision":
                    None,
            }
        )

    # ================================================================
    # Runtime events
    # ================================================================

    def _runtime_event_callback(
        self,
        message: String,
    ) -> None:

        event = self._extract_event(
            message.data
        )

        if not event:
            return

        if event not in RUNTIME_EVENT_UPDATES:

            self.get_logger().warning(
                f"Unsupported runtime event: "
                f"{event}"
            )
            return

        if event == "GPS_LOST":

            if not self._is_navigating():
                return

            if self.recovery_active:
                return

        elif event in {
            "GPS_AVAILABLE",
            "GPS_RECOVERY_FAILED",
        }:

            if (
                not self.recovery_active
                or self.recovery_action
                != "WAIT_FOR_GPS"
            ):
                return

        self.get_logger().warning(
            f"RUNTIME EVENT RECEIVED: "
            f"{event}"
        )

        self.rule_engine.update_facts(
            RUNTIME_EVENT_UPDATES[
                event
            ]
        )

        result = (
            self.rule_engine.inference_cycle()
        )

        self._log_inference(
            result
        )

        decision = (
            self.working_memory.get(
                "decision"
            )
        )

        if decision == "WAIT_FOR_GPS":

            self._start_gps_wait()
            return

        if decision == "RESUME_MISSION":

            self._resume_mission()
            return

        if decision == "EMERGENCY_LAND":

            self._start_emergency_land()
            return

    # ================================================================
    # GPS recovery
    # ================================================================

    def _start_gps_wait(
        self,
    ) -> None:

        self.recovery_active = True
        self.recovery_action = (
            "WAIT_FOR_GPS"
        )

        self.get_logger().warning(
            "ES RECOVERY RUNNING: "
            "WAIT_FOR_GPS"
        )

        self._publish_progress(
            state_name="WAIT_FOR_GPS",
            task_type="recovery",
            status="RUNNING",
        )

    def _resume_mission(
        self,
    ) -> None:

        self.recovery_active = False
        self.recovery_action = None

        state_name = (
            self._task_label(
                self.current_task,
                self.current_task_index,
            )
        )

        self.get_logger().info(
            f"ES RESUMED: "
            f"{state_name}"
        )

        self._publish_progress(
            state_name=state_name,
            task_type=self._task_type(
                self.current_task
            ),
            status="RUNNING",
        )

    def _start_emergency_land(
        self,
    ) -> None:

        self.recovery_active = True
        self.recovery_action = (
            "EMERGENCY_LAND"
        )

        self.termination_reason = str(
            self.working_memory.get(
                "termination_reason",
                "GPS_RECOVERY_FAILED",
            )
        )

        self.get_logger().warning(
            "ES RECOVERY RUNNING: "
            "EMERGENCY_LAND"
        )

        self._publish_progress(
            state_name="EMERGENCY_LAND",
            task_type="recovery",
            status="RUNNING",
        )

    def _tick_recovery(
        self,
    ) -> None:

        if (
            self.recovery_action
            == "WAIT_FOR_GPS"
        ):
            return

        if (
            self.recovery_action
            == "EMERGENCY_LAND"
        ):

            self._publish_progress(
                state_name="EMERGENCY_LAND",
                task_type="recovery",
                status="COMPLETED",
            )

            self.get_logger().warning(
                "ES RECOVERY COMPLETED: "
                "EMERGENCY_LAND"
            )

            self.rule_engine.update_facts(
                {
                    "safe_termination_ready":
                        True,

                    "decision":
                        None,
                }
            )

            result = (
                self.rule_engine.inference_cycle()
            )

            self._log_inference(
                result
            )

            if (
                self.working_memory.get(
                    "decision"
                )
                == "SAFE_TERMINATE"
            ):

                self._finish_safe()

    # ================================================================
    # Helpers
    # ================================================================

    def _is_navigating(
        self,
    ) -> bool:

        return (
            self.current_task_running
            and self.current_task
            is not None
            and self._task_type(
                self.current_task
            )
            == "navigate"
        )

    def _task_type(
        self,
        task: dict,
    ) -> str:

        return str(
            task.get(
                "type",
                "",
            )
        ).lower()

    def _task_label(
        self,
        task: dict,
        index: int,
    ) -> str:

        task_type = self._task_type(
            task
        )

        if task_type == "navigate":

            waypoint = task.get(
                "waypoint",
                task.get(
                    "name",
                    index,
                ),
            )

            return (
                f"NAVIGATE_{index}_"
                f"{waypoint}"
            )

        return (
            f"{task_type.upper()}_"
            f"{index}"
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

    def _log_inference(
        self,
        result,
    ) -> None:

        self.get_logger().info(
            f"INFERENCE CYCLE: "
            f"{result.cycle}"
        )

        self.get_logger().info(
            f"Conflict Set: "
            f"{result.conflict_set}"
        )

        self.get_logger().info(
            f"Selected Rule: "
            f"{result.selected_rule}"
        )

        self.get_logger().info(
            f"Decision: "
            f"{self.working_memory.get('decision')}"
        )

    # ================================================================
    # Progress publication
    # ================================================================

    def _publish_progress(
        self,
        state_name: str,
        task_type: str,
        status: str,
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
                    status,

                "completed_navigation_tasks":
                    self.completed_navigation_tasks,
            }
        )

        self.progress_publisher.publish(
            message
        )

    # ================================================================
    # Mission completion
    # ================================================================

    def _complete_mission(
        self,
    ) -> None:

        self.rule_engine.update_facts(
            {
                "current_task_type":
                    None,

                "current_task_index":
                    None,

                "all_tasks_processed":
                    True,

                "decision":
                    None,
            }
        )

        result = (
            self.rule_engine.inference_cycle()
        )

        if (
            result.selected_rule
            != "r_mission_complete"
        ):
            raise RuntimeError(
                "Mission completion rule "
                "was not selected."
            )

        self._log_inference(
            result
        )

        self.mission_finished = True
        self.timer.cancel()

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "MISSION COMPLETED"
        )

        self.get_logger().info(
            f"Completed tasks: "
            f"{self.completed_tasks}"
        )

        self.get_logger().info(
            f"Completed navigation tasks: "
            f"{self.completed_navigation_tasks}"
        )

        self.get_logger().info(
            f"Inference cycles: "
            f"{self.rule_engine.cycle_count}"
        )

        self.get_logger().info(
            f"Rules fired: "
            f"{self.rule_engine.fire_count}"
        )

        self.get_logger().info(
            "========================================"
        )

    def _finish_safe(
        self,
    ) -> None:

        if self.mission_finished:
            return

        self.mission_finished = True
        self.recovery_active = False
        self.recovery_action = None

        self.timer.cancel()

        self.get_logger().warning(
            "========================================"
        )

        self.get_logger().warning(
            "MISSION SAFE TERMINATED"
        )

        self.get_logger().warning(
            f"Reason: "
            f"{self.termination_reason}"
        )

        self.get_logger().warning(
            "========================================"
        )


def main(
    args=None,
) -> None:

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = ExpertSystemExecutor()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
