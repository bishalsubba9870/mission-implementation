#!/usr/bin/env python3

import json
import os
import re

import py_trees
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from mission_formalism_evaluation.bt.tree_builder import GPSTreeBuilder


class BehaviorTreeExecutor(Node):

    def __init__(self):

        super().__init__("bt_executor")

        self.declare_parameter(
            "mission_file",
            "mission_1.yaml",
        )

        self.declare_parameter(
            "state_duration",
            0.2,
        )

        self.declare_parameter(
            "tick_period",
            0.02,
        )

        self.mission_file = self.get_parameter(
            "mission_file"
        ).value

        self.state_duration = float(
            self.get_parameter(
                "state_duration"
            ).value
        )

        self.tick_period = float(
            self.get_parameter(
                "tick_period"
            ).value
        )

        # ==========================================================
        # ROS
        # ==========================================================

        self.progress_pub = self.create_publisher(
            String,
            "/mission/progress",
            10,
        )

        self.event_sub = self.create_subscription(
            String,
            "/mission/event",
            self.event_callback,
            10,
        )

        # ==========================================================
        # Mission
        # ==========================================================

        self.mission_id = ""
        self.mission_name = ""

        self.tasks = []

        self.completed_tasks = 0
        self.completed_navigation_tasks = 0

        self.current_task = None

        self.finished = False
        self.safe_terminated = False
        self.termination_reason = None

        # ==========================================================
        # GPS
        # ==========================================================

        self.gps_recovery_active = False
        self.gps_recovery_result = None
        self.gps_emergency_landing = False

        # ==========================================================
        # Propulsion
        # ==========================================================

        self.propulsion_failure_active = False

        # ==========================================================
        # Communication
        # ==========================================================

        self.communication_loss_active = False
        self.communication_recovery_result = None
        self.communication_rtl_active = False

        # ==========================================================
        # Wind
        # ==========================================================

        self.wind_unsafe_active = False
        self.wind_safe_landing_active = False

        # ==========================================================
        # Battery
        # ==========================================================

        self.battery_critical_active = False

        # ==========================================================
        # Mission loading
        # ==========================================================

        mission_path = self.resolve_mission_file(
            self.mission_file
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Behavior Tree Mission Executor"
        )

        self.get_logger().info(
            "Mode: GPS + PROPULSION + "
            "COMMUNICATION + WIND + BATTERY"
        )

        self.get_logger().info(
            "========================================"
        )

        self.load_mission(
            mission_path
        )

        builder = GPSTreeBuilder(
            executor=self,
            task_duration=self.state_duration,
        )

        self.tree = builder.build(
            self.tasks
        )

        self.print_tree_structure()

        self.timer = self.create_timer(
            self.tick_period,
            self.tick_tree,
        )

    # ==============================================================
    # Mission file
    # ==============================================================

    def resolve_mission_file(
        self,
        mission_file,
    ):

        if os.path.isabs(mission_file):

            if os.path.isfile(mission_file):
                return mission_file

            raise FileNotFoundError(
                mission_file
            )

        share_dir = get_package_share_directory(
            "mission_formalism_evaluation"
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

            if os.path.isfile(path):
                return path

        raise FileNotFoundError(
            mission_file
        )

    # ==============================================================
    # Mission loading
    # ==============================================================

    def load_mission(
        self,
        mission_path,
    ):

        with open(
            mission_path,
            "r",
            encoding="utf-8",
        ) as file:

            data = yaml.safe_load(
                file
            )

        if not isinstance(data, dict):

            raise ValueError(
                "Mission YAML root must be a dictionary."
            )

        raw_mission_id = str(
            data.get(
                "mission_id",
                data.get(
                    "id",
                    os.path.splitext(
                        os.path.basename(
                            mission_path
                        )
                    )[0],
                ),
            )
        )

        match = re.fullmatch(
            r"mission_(\d+)",
            raw_mission_id,
            re.IGNORECASE,
        )

        if match:
            self.mission_id = (
                f"M{match.group(1)}"
            )

        else:
            self.mission_id = (
                raw_mission_id.upper()
            )

        self.mission_name = str(
            data.get(
                "mission_name",
                data.get(
                    "name",
                    raw_mission_id,
                ),
            )
        )

        raw_tasks = self.find_task_list(
            data
        )

        self.tasks = [
            self.normalize_task(
                index,
                raw_task,
            )
            for index, raw_task
            in enumerate(
                raw_tasks,
                start=1,
            )
        ]

        self.get_logger().info(
            f"Mission ID: {self.mission_id}"
        )

        self.get_logger().info(
            f"Mission items: {len(self.tasks)}"
        )

    @staticmethod
    def find_task_list(data):

        for key in [
            "tasks",
            "mission_items",
            "actions",
            "steps",
            "mission",
        ]:

            value = data.get(key)

            if isinstance(value, list):
                return value

        raise ValueError(
            "No mission task list found."
        )

    def normalize_task(
        self,
        index,
        raw_task,
    ):

        if isinstance(raw_task, str):

            raw = {
                "name": raw_task
            }

        elif isinstance(raw_task, dict):

            raw = dict(raw_task)

        else:

            raise ValueError(
                f"Invalid task at index {index}"
            )

        task_id = str(
            raw.get(
                "id",
                raw.get(
                    "task_id",
                    f"TASK_{index}",
                ),
            )
        )

        name = str(
            raw.get(
                "name",
                raw.get(
                    "description",
                    task_id,
                ),
            )
        )

        task_type = self.determine_task_type(
            raw,
            task_id,
            name,
        )

        target = str(
            raw.get(
                "target",
                raw.get(
                    "waypoint",
                    raw.get(
                        "location",
                        "",
                    ),
                ),
            )
        )

        if (
            task_type == "navigate"
            and not target
        ):

            target = self.extract_waypoint(
                f"{task_id} {name}"
            )

        return {
            "index": index,
            "id": task_id,
            "type": task_type,
            "name": name,
            "target": target,
        }

    def determine_task_type(
        self,
        raw,
        task_id,
        name,
    ):

        explicit_type = raw.get(
            "type",
            raw.get(
                "task_type",
                raw.get(
                    "action",
                    "",
                ),
            ),
        )

        if explicit_type:

            return self.normalize_task_type(
                explicit_type
            )

        text = (
            f"{task_id} {name}"
        ).lower()

        if "takeoff" in text:
            return "takeoff"

        if "land" in text:
            return "land"

        if "hover" in text:
            return "hover"

        if "inspect" in text:
            return "inspect"

        if (
            "rtl" in text
            or "return_to_home" in text
        ):
            return "rtl"

        if (
            "navigate" in text
            or "waypoint" in text
            or re.search(
                r"\bwp\s*[0-9]+\b",
                text,
            )
        ):
            return "navigate"

        return "task"

    @staticmethod
    def normalize_task_type(value):

        value = str(value).strip().lower()

        aliases = {
            "take_off": "takeoff",
            "take-off": "takeoff",
            "nav": "navigate",
            "navigation": "navigate",
            "goto": "navigate",
            "go_to": "navigate",
            "waypoint": "navigate",
            "inspection": "inspect",
            "landing": "land",
            "return_to_home": "rtl",
        }

        return aliases.get(
            value,
            value,
        )

    @staticmethod
    def extract_waypoint(text):

        match = re.search(
            r"\bWP\s*([0-9]+)\b",
            str(text),
            re.IGNORECASE,
        )

        if match:
            return f"WP{match.group(1)}"

        return ""

    # ==============================================================
    # Runtime events
    # ==============================================================

    def event_callback(
        self,
        msg,
    ):

        raw_message = msg.data.strip()

        event = ""
        event_mission = ""

        try:

            event_data = json.loads(
                raw_message
            )

            if isinstance(event_data, dict):

                event = str(
                    event_data.get(
                        "event",
                        "",
                    )
                ).strip()

                event_mission = str(
                    event_data.get(
                        "mission_id",
                        "",
                    )
                ).upper()

            elif isinstance(event_data, str):

                event = event_data.strip()

            else:

                event = str(
                    event_data
                ).strip()

        except json.JSONDecodeError:

            event = raw_message

        if not event:
            return

        if (
            event_mission
            and event_mission
            != self.mission_id.upper()
        ):
            return

        self.get_logger().warning(
            f"RUNTIME EVENT RECEIVED: {event}"
        )

        if self.finished:
            return

        # ==========================================================
        # GPS
        # ==========================================================

        if event == "GPS_LOST":

            if (
                self.current_task is None
                or self.current_task["type"]
                != "navigate"
            ):
                return

            self.gps_recovery_active = True
            self.gps_recovery_result = None

            self.get_logger().info(
                "BT GPS RECOVERY ACTIVE"
            )

            return

        if event == "GPS_AVAILABLE":

            if self.gps_recovery_active:
                self.gps_recovery_result = (
                    "GPS_AVAILABLE"
                )

            return

        if event == "GPS_RECOVERY_FAILED":

            if self.gps_recovery_active:
                self.gps_recovery_result = (
                    "GPS_RECOVERY_FAILED"
                )

            return

        # ==========================================================
        # Propulsion
        # ==========================================================

        if event == "PROPULSION_FAILURE":

            self.propulsion_failure_active = True

            self.get_logger().error(
                "PROPULSION FAILURE DETECTED"
            )

            return

        # ==========================================================
        # Communication
        # ==========================================================

        if event == "COMMUNICATION_LOST":

            if self.communication_loss_active:
                return

            self.communication_loss_active = True
            self.communication_recovery_result = None

            self.get_logger().warning(
                "COMMUNICATION LOST"
            )

            self.get_logger().info(
                "BT COMMUNICATION RECOVERY ACTIVE"
            )

            return

        if event == "COMMUNICATION_RESTORED":

            if self.communication_loss_active:
                self.communication_recovery_result = (
                    "COMMUNICATION_RESTORED"
                )

            return

        if event == "COMMUNICATION_RECOVERY_FAILED":

            if self.communication_loss_active:
                self.communication_recovery_result = (
                    "COMMUNICATION_RECOVERY_FAILED"
                )

            return

        # ==========================================================
        # Wind
        # ==========================================================

        if event == "WIND_UNSAFE":

            if self.wind_unsafe_active:
                return

            self.wind_unsafe_active = True

            self.get_logger().warning(
                "UNSAFE WIND DETECTED"
            )

            return

        # ==========================================================
        # Battery critical
        # ==========================================================

        if event == "BATTERY_CRITICAL":

            if self.battery_critical_active:
                return

            self.battery_critical_active = True

            self.get_logger().error(
                "BATTERY CRITICAL DETECTED"
            )

            self.get_logger().warning(
                "Normal mission execution interrupted."
            )

            return

        self.get_logger().warning(
            f"Unsupported event: {event}"
        )

    # ==============================================================
    # BT tick
    # ==============================================================

    def tick_tree(self):

        if self.finished:
            return

        self.tree.tick()

        if self.safe_terminated:

            self.finished = True

            self.publish_mission_status(
                "SAFE_TERMINATED"
            )

            self.get_logger().info(
                "========================================"
            )

            self.get_logger().info(
                "MISSION SAFE TERMINATED"
            )

            self.get_logger().info(
                f"Reason: {self.termination_reason}"
            )

            self.get_logger().info(
                f"Completed tasks: {self.completed_tasks}"
            )

            self.get_logger().info(
                f"Completed navigation tasks: "
                f"{self.completed_navigation_tasks}"
            )

            self.get_logger().info(
                "========================================"
            )

            self.timer.cancel()

            return

        status = self.tree.root.status

        # Continuous RUNNING progress publication
        if (
            status == py_trees.common.Status.RUNNING
            and self.current_task is not None
        ):

            self.publish_task_progress(
                self.current_task,
                "RUNNING",
            )

        if status == py_trees.common.Status.SUCCESS:

            self.finished = True

            self.publish_mission_status(
                "COMPLETED"
            )

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
                "========================================"
            )

            self.timer.cancel()

        elif status == py_trees.common.Status.FAILURE:

            self.finished = True

            self.publish_mission_status(
                "FAILED"
            )

            self.get_logger().error(
                "MISSION FAILED"
            )

            self.timer.cancel()

    # ==============================================================
    # Mission callbacks
    # ==============================================================

    def on_task_started(
        self,
        task,
    ):

        self.current_task = task

        self.get_logger().info(
            "----------------------------------------"
        )

        self.get_logger().info(
            f"BT NODE RUNNING: "
            f"{self.node_name(task)}"
        )

        self.get_logger().info(
            f"ACTION: "
            f"{self.action_description(task)}"
        )

        self.publish_task_progress(
            task,
            "RUNNING",
        )

    def on_task_resumed(
        self,
        task,
    ):

        self.current_task = task

        self.get_logger().info(
            f"BT NODE RESUMED: "
            f"{self.node_name(task)}"
        )

        self.publish_task_progress(
            task,
            "RUNNING",
        )

    def on_task_paused(
        self,
        task,
        reason,
    ):

        self.get_logger().warning(
            f"BT NODE PAUSED: "
            f"{self.node_name(task)} "
            f"| Reason: {reason}"
        )

    def on_gps_recovered(self):

        if self.current_task is not None:

            self.get_logger().info(
                f"GPS RECOVERED -> resume "
                f"{self.node_name(self.current_task)}"
            )

    def on_communication_recovered(self):

        if self.current_task is not None:

            self.get_logger().info(
                f"COMMUNICATION RECOVERED -> resume "
                f"{self.node_name(self.current_task)}"
            )

    def on_task_completed(
        self,
        task,
    ):

        self.completed_tasks += 1

        if task["type"] == "navigate":
            self.completed_navigation_tasks += 1

        self.get_logger().info(
            f"BT NODE SUCCESS: "
            f"{self.node_name(task)}"
        )

        self.get_logger().info(
            f"Completed tasks: "
            f"{self.completed_tasks}/"
            f"{len(self.tasks)}"
        )

        self.get_logger().info(
            f"Completed navigation tasks: "
            f"{self.completed_navigation_tasks}"
        )

        self.publish_task_progress(
            task,
            "COMPLETED",
        )

    # ==============================================================
    # Termination
    # ==============================================================

    def mark_safe_terminated(
        self,
        reason,
    ):

        self.safe_terminated = True
        self.termination_reason = reason
        self.current_task = None

    # ==============================================================
    # Naming
    # ==============================================================

    @staticmethod
    def node_name(task):

        if task["type"] == "navigate":

            return (
                f"NAV_"
                f"{task['index']}_"
                f"{task['target']}"
            )

        return (
            f"{task['type'].upper()}_"
            f"{task['index']}"
        )

    @staticmethod
    def action_description(task):

        task_type = task["type"]

        if task_type == "takeoff":
            return "TAKEOFF"

        if task_type == "navigate":
            return f"NAVIGATE -> {task['target']}"

        if task_type == "hover":
            return "HOVER"

        if task_type == "inspect":
            return "INSPECT"

        if task_type == "land":
            return "LAND"

        if task_type == "rtl":
            return "RETURN_TO_HOME"

        return task["name"]

    # ==============================================================
    # Common progress interface
    # ==============================================================

    def publish_task_progress(
        self,
        task,
        status,
    ):

        payload = {
            "mission_id":
                self.mission_id,

            "state_name":
                self.node_name(task),

            "completed_navigation_tasks":
                self.completed_navigation_tasks,

            "task_type":
                task["type"],

            "task_status":
                status,

            "formalism":
                "BT",

            "task_index":
                task["index"],

            "task_id":
                task["id"],

            "target":
                task["target"],

            "completed_task_count":
                self.completed_tasks,
        }

        msg = String()

        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )

        self.progress_pub.publish(
            msg
        )

    def publish_mission_status(
        self,
        status,
    ):

        payload = {
            "mission_id":
                self.mission_id,

            "formalism":
                "BT",

            "mission_status":
                status,

            "completed_task_count":
                self.completed_tasks,

            "completed_navigation_tasks":
                self.completed_navigation_tasks,

            "termination_reason":
                self.termination_reason,
        }

        msg = String()

        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )

        self.progress_pub.publish(
            msg
        )

    # ==============================================================
    # Structural output
    # ==============================================================

    def print_tree_structure(self):

        nodes = list(
            self.tree.root.iterate()
        )

        total_nodes = len(nodes)

        leaf_nodes = sum(
            1
            for node in nodes
            if not node.children
        )

        composite_nodes = (
            total_nodes - leaf_nodes
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "BEHAVIOR TREE STRUCTURE"
        )

        self.get_logger().info(
            f"BT total nodes: {total_nodes}"
        )

        self.get_logger().info(
            f"BT composite nodes: {composite_nodes}"
        )

        self.get_logger().info(
            f"BT leaf/action nodes: {leaf_nodes}"
        )

        self.get_logger().info(
            "Runtime controllers: "
            "GPS + Propulsion + Communication + Wind + Battery"
        )

        self.get_logger().info(
            "========================================"
        )


def main(args=None):

    rclpy.init(args=args)

    node = BehaviorTreeExecutor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
