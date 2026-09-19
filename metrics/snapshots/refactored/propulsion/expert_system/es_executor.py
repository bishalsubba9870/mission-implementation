"""GPS and propulsion-capable Expert System executor."""

from __future__ import annotations

import json

import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from mission_formalism_evaluation.common.mission_loader import (
    load_mission,
)
from mission_formalism_evaluation.common.runtime_condition import (
    parse_runtime_event,
)
from mission_formalism_evaluation.expert_system.es_domain import (
    RUNTIME_EVENT_UPDATES,
    build_knowledge_base,
)
from mission_formalism_evaluation.expert_system.rule_engine import (
    InferenceResult,
    RuleEngine,
    WorkingMemory,
)


PACKAGE_NAME = "mission_formalism_evaluation"


class ExpertSystemExecutor(Node):
    """Execute missions using production rules."""

    def __init__(self) -> None:

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

        (
            self.mission_id,
            self.mission_name,
            self.tasks,
            self.mission_path,
        ) = load_mission(
            PACKAGE_NAME,
            self.mission_file,
        )

        self.current_task_index = 0
        self.current_task: dict | None = None
        self.current_task_running = False

        self.completed_tasks = 0
        self.completed_navigation_tasks = 0

        self.mission_finished = False

        self.recovery_active = False
        self.recovery_action: str | None = None
        self.termination_reason = ""

        self.progress_publisher = self.create_publisher(
            String,
            "/mission/progress",
            10,
        )

        self.event_subscription = self.create_subscription(
            String,
            "/mission/event",
            self._runtime_event_callback,
            10,
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
                "propulsion_failure": False,
                "safe_termination_ready": False,
                "safe_terminated": False,
                "termination_reason": "",
            }
        )

        self.knowledge_base = build_knowledge_base()

        self.rule_engine = RuleEngine(
            rules=self.knowledge_base,
            working_memory=self.working_memory,
        )

        self._log_startup()

        self.timer = self.create_timer(
            self.state_duration,
            self._timer_callback,
        )

    def _timer_callback(self) -> None:
        """Advance mission or recovery execution."""

        if self.mission_finished:
            return

        if self.recovery_active:
            self._tick_recovery()
            return

        if self.current_task_running:
            self._complete_current_task()
            return

        if self.current_task_index >= len(
            self.tasks
        ):
            self._complete_mission()
            return

        self._start_current_task()

    def _start_current_task(self) -> None:
        """Select a rule for the current mission task."""

        self.current_task = self.tasks[
            self.current_task_index
        ]

        task_type = self._task_type(
            self.current_task
        )

        self.rule_engine.update_facts(
            {
                "current_task_type": task_type,
                "current_task_index":
                    self.current_task_index,
                "all_tasks_processed": False,
                "decision": None,
            }
        )

        result = self.rule_engine.inference_cycle()

        if result.selected_rule is None:
            raise RuntimeError(
                "No production rule matched "
                f"task {self.current_task_index}: "
                f"{task_type}"
            )

        self._log_inference(result)

        state_name = self._task_label(
            self.current_task,
            self.current_task_index,
        )

        self.get_logger().info(
            f"ES RUNNING: {state_name}"
        )

        self.current_task_running = True

        self._publish_progress(
            state_name=state_name,
            task_type=task_type,
            status="RUNNING",
        )

    def _complete_current_task(self) -> None:
        """Complete current mission task."""

        if self.current_task is None:
            return

        task_type = self._task_type(
            self.current_task
        )

        state_name = self._task_label(
            self.current_task,
            self.current_task_index,
        )

        self._publish_progress(
            state_name=state_name,
            task_type=task_type,
            status="COMPLETED",
        )

        self.get_logger().info(
            f"ES COMPLETED: {state_name}"
        )

        if task_type == "navigate":
            self.completed_navigation_tasks += 1

        self.completed_tasks += 1
        self.current_task_index += 1

        self.current_task = None
        self.current_task_running = False

        self.rule_engine.update_facts(
            {
                "current_task_type": None,
                "current_task_index": None,
                "decision": None,
            }
        )

    def _runtime_event_callback(
        self,
        message: String,
    ) -> None:
        """Handle supported runtime events."""

        event = parse_runtime_event(
            message.data
        )

        if not event:
            return

        if event not in RUNTIME_EVENT_UPDATES:
            self.get_logger().warning(
                f"Unsupported runtime event: {event}"
            )
            return

        if event in {
            "GPS_LOST",
            "PROPULSION_FAILURE",
        }:
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
            f"RUNTIME EVENT RECEIVED: {event}"
        )

        self.rule_engine.update_facts(
            RUNTIME_EVENT_UPDATES[event]
        )

        result = self.rule_engine.inference_cycle()

        self._log_inference(
            result
        )

        decision = self.working_memory.get(
            "decision"
        )

        if decision == "WAIT_FOR_GPS":
            self._start_gps_wait()
            return

        if decision == "RESUME_MISSION":
            self._resume_mission()
            return

        if decision == "EMERGENCY_LAND":
            self._start_emergency_land()

    def _start_gps_wait(self) -> None:
        """Wait for GPS recovery."""

        self.recovery_active = True
        self.recovery_action = "WAIT_FOR_GPS"

        self.get_logger().warning(
            "ES RECOVERY RUNNING: WAIT_FOR_GPS"
        )

        self._publish_progress(
            state_name="WAIT_FOR_GPS",
            task_type="recovery",
            status="RUNNING",
        )

    def _resume_mission(self) -> None:
        """Resume interrupted navigation."""

        self.recovery_active = False
        self.recovery_action = None

        state_name = self._task_label(
            self.current_task,
            self.current_task_index,
        )

        self.get_logger().info(
            f"ES RESUMED: {state_name}"
        )

        self._publish_progress(
            state_name=state_name,
            task_type=self._task_type(
                self.current_task
            ),
            status="RUNNING",
        )

    def _start_emergency_land(self) -> None:
        """Start emergency landing."""

        self.recovery_active = True
        self.recovery_action = "EMERGENCY_LAND"

        self.termination_reason = str(
            self.working_memory.get(
                "termination_reason",
                "",
            )
        )

        self.get_logger().warning(
            "ES RECOVERY RUNNING: EMERGENCY_LAND"
        )

        self._publish_progress(
            state_name="EMERGENCY_LAND",
            task_type="recovery",
            status="RUNNING",
        )

    def _tick_recovery(self) -> None:
        """Advance recovery action."""

        if self.recovery_action == "WAIT_FOR_GPS":
            return

        if self.recovery_action == "EMERGENCY_LAND":
            self._finish_emergency_land()

    def _finish_emergency_land(self) -> None:
        """Complete emergency landing."""

        self._publish_progress(
            state_name="EMERGENCY_LAND",
            task_type="recovery",
            status="COMPLETED",
        )

        self.get_logger().warning(
            "ES RECOVERY COMPLETED: EMERGENCY_LAND"
        )

        self.recovery_active = False
        self.recovery_action = None

        self.rule_engine.update_facts(
            {
                "safe_termination_ready": True,
                "decision": None,
            }
        )

        result = self.rule_engine.inference_cycle()

        self._log_inference(result)

        if (
            self.working_memory.get("decision")
            == "SAFE_TERMINATE"
        ):
            self._finish_safe()

    def _finish_safe(self) -> None:
        """Finish after safe termination."""

        self.mission_finished = True
        self.timer.cancel()

        self.get_logger().warning(
            "MISSION SAFELY TERMINATED"
        )
        self.get_logger().warning(
            f"Reason: {self.termination_reason}"
        )
        self.get_logger().warning(
            f"Completed tasks: {self.completed_tasks}"
        )

    def _complete_mission(self) -> None:
        """Fire normal mission-completion rule."""

        self.rule_engine.update_facts(
            {
                "current_task_type": None,
                "current_task_index": None,
                "all_tasks_processed": True,
                "decision": None,
            }
        )

        result = self.rule_engine.inference_cycle()

        if (
            result.selected_rule
            != "r_mission_complete"
        ):
            raise RuntimeError(
                "Mission completion rule "
                "was not selected."
            )

        self._log_inference(result)

        self.mission_finished = True
        self.timer.cancel()

        self.get_logger().info(
            "========================================"
        )
        self.get_logger().info(
            "MISSION COMPLETED"
        )
        self.get_logger().info(
            f"Completed tasks: {self.completed_tasks}"
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

    def _is_navigating(self) -> bool:
        """Return whether navigation is active."""

        return (
            self.current_task_running
            and self.current_task is not None
            and self._task_type(
                self.current_task
            )
            == "navigate"
        )

    @staticmethod
    def _task_type(
        task: dict | None,
    ) -> str:
        """Return normalized task type."""

        if task is None:
            return ""

        return str(
            task.get(
                "type",
                "",
            )
        ).lower()

    def _task_label(
        self,
        task: dict | None,
        index: int,
    ) -> str:
        """Return readable task label."""

        if task is None:
            return "NONE"

        task_type = self._task_type(task)

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
            f"{task_type.upper()}_{index}"
        )

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
                "mission_id": self.mission_id,
                "state_name": state_name,
                "task_type": task_type,
                "task_status": status,
                "completed_navigation_tasks":
                    self.completed_navigation_tasks,
            }
        )

        self.progress_publisher.publish(
            message
        )

    def _log_inference(
        self,
        result: InferenceResult,
    ) -> None:
        """Log production-rule inference."""

        decision = self.working_memory.get(
            "decision"
        )

        self.get_logger().info(
            f"INFERENCE CYCLE: {result.cycle}"
        )
        self.get_logger().info(
            f"Conflict Set: {result.conflict_set}"
        )
        self.get_logger().info(
            f"Selected Rule: {result.selected_rule}"
        )
        self.get_logger().info(
            f"Decision: {decision}"
        )

    def _log_startup(self) -> None:
        """Log Expert System configuration."""

        self.get_logger().info(
            f"Loading mission: {self.mission_path}"
        )
        self.get_logger().info(
            "========================================"
        )
        self.get_logger().info(
            "Expert System Mission Executor"
        )
        self.get_logger().info(
            "Mode: GPS + PROPULSION"
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


def main(
    args=None,
) -> None:
    """Run the Expert System executor."""

    rclpy.init(args=args)

    node = None

    try:
        node = ExpertSystemExecutor()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
