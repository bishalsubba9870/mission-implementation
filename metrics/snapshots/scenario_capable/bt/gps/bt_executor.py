#!/usr/bin/env python3

import json
import os
import re

import py_trees
import rclpy
import yaml

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from std_msgs.msg import String

from mission_formalism_evaluation.bt.tree_builder import (
    GPSTreeBuilder,
)


class BehaviorTreeExecutor(Node):
    """
    GPS-capable Behavior Tree mission executor.

    Supported runtime events:

        GPS_LOST
        GPS_AVAILABLE
        GPS_RECOVERY_FAILED

    No propulsion, communication, battery, wind,
    or operator-abort behaviour is implemented yet.
    """

    def __init__(
        self,
    ):

        super().__init__(
            "bt_executor"
        )

        # ==========================================================
        # Parameters
        # ==========================================================

        self.declare_parameter(
            "mission_file",
            "mission_1.yaml",
        )

        self.declare_parameter(
            "state_duration",
            0.1,
        )

        self.declare_parameter(
            "tick_period",
            0.02,
        )

        self.mission_file = (
            self.get_parameter(
                "mission_file"
            ).value
        )

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
        # ROS interfaces
        # ==========================================================

        self.progress_pub = (
            self.create_publisher(
                String,
                "/mission/progress",
                10,
            )
        )

        self.event_sub = (
            self.create_subscription(
                String,
                "/mission/event",
                self.event_callback,
                10,
            )
        )

        # ==========================================================
        # Mission state
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
        # GPS runtime state
        # ==========================================================

        self.gps_recovery_active = False

        self.gps_recovery_result = None

        self.gps_emergency_landing = False

        # ==========================================================
        # Mission loading
        # ==========================================================

        mission_path = (
            self.resolve_mission_file(
                self.mission_file
            )
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Behavior Tree Mission Executor"
        )

        self.get_logger().info(
            "Mode: GPS_CAPABLE"
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            f"Mission file: {mission_path}"
        )

        self.load_mission(
            mission_path
        )

        # ==========================================================
        # Build GPS-capable tree
        # ==========================================================

        builder = GPSTreeBuilder(
            executor=self,
            task_duration=(
                self.state_duration
            ),
        )

        self.tree = builder.build(
            self.tasks
        )

        self.print_tree_structure()

        # ==========================================================
        # Timer
        # ==========================================================

        self.timer = (
            self.create_timer(
                self.tick_period,
                self.tick_tree,
            )
        )

        self.get_logger().info(
            f"Logical task duration: "
            f"{self.state_duration:.3f} s"
        )

        self.get_logger().info(
            f"BT tick period: "
            f"{self.tick_period:.3f} s"
        )

        self.get_logger().info(
            "GPS-capable Behavior Tree started."
        )

    # ==============================================================
    # Mission file
    # ==============================================================

    def resolve_mission_file(
        self,
        mission_file,
    ):

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
            f"Mission file not found: "
            f"{mission_file}"
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

        if not isinstance(
            data,
            dict,
        ):

            raise ValueError(
                "Mission YAML root must "
                "be a dictionary."
            )

        self.mission_id = str(

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

        self.mission_name = str(

            data.get(
                "mission_name",

                data.get(
                    "name",
                    self.mission_id,
                ),
            )
        )

        raw_tasks = (
            self.find_task_list(
                data
            )
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

        if not self.tasks:

            raise ValueError(
                "Mission has zero tasks."
            )

        self.get_logger().info(
            f"Mission ID: "
            f"{self.mission_id}"
        )

        self.get_logger().info(
            f"Mission name: "
            f"{self.mission_name}"
        )

        self.get_logger().info(
            f"Mission items: "
            f"{len(self.tasks)}"
        )

    @staticmethod
    def find_task_list(
        data,
    ):

        for key in [

            "tasks",
            "mission_items",
            "actions",
            "steps",
            "mission",

        ]:

            value = data.get(
                key
            )

            if isinstance(
                value,
                list,
            ):

                return value

        raise ValueError(
            "No mission task list found."
        )

    def normalize_task(
        self,
        index,
        raw_task,
    ):

        if isinstance(
            raw_task,
            str,
        ):

            raw = {
                "name": raw_task
            }

        elif isinstance(
            raw_task,
            dict,
        ):

            raw = dict(
                raw_task
            )

        else:

            raise ValueError(
                f"Invalid task at "
                f"index {index}"
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

        task_type = (
            self.determine_task_type(
                raw,
                task_id,
                name,
            )
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

            target = (
                self.extract_waypoint(
                    f"{task_id} {name}"
                )
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

            return (
                self.normalize_task_type(
                    explicit_type
                )
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
            or "return_to_home"
            in text
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
    def normalize_task_type(
        value,
    ):

        value = str(
            value
        ).strip().lower()

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
    def extract_waypoint(
        text,
    ):

        match = re.search(

            r"\bWP\s*([0-9]+)\b",

            str(text),

            re.IGNORECASE,
        )

        if match:

            return (
                f"WP{match.group(1)}"
            )

        return ""

    # ==============================================================
    # Runtime events
    # ==============================================================

    def event_callback(
        self,
        msg,
    ):

        event = (
            msg.data.strip()
        )

        self.get_logger().warning(
            f"RUNTIME EVENT RECEIVED: "
            f"{event}"
        )

        if self.finished:
            return

        # ----------------------------------------------------------
        # GPS lost
        # ----------------------------------------------------------

        if event == "GPS_LOST":

            if self.current_task is None:

                self.get_logger().warning(
                    "GPS_LOST ignored: "
                    "no active mission task."
                )

                return

            if (
                self.current_task["type"]
                != "navigate"
            ):

                self.get_logger().warning(
                    "GPS_LOST ignored: "
                    "current task is not navigation."
                )

                return

            if self.gps_recovery_active:

                return

            self.gps_recovery_active = (
                True
            )

            self.gps_recovery_result = (
                None
            )

            self.get_logger().warning(
                f"GPS LOST during "
                f"{self.node_name(self.current_task)}"
            )

            self.get_logger().info(
                "BT GPS RECOVERY ACTIVE"
            )

            return

        # ----------------------------------------------------------
        # GPS available
        # ----------------------------------------------------------

        if event == "GPS_AVAILABLE":

            if self.gps_recovery_active:

                self.gps_recovery_result = (
                    "GPS_AVAILABLE"
                )

            return

        # ----------------------------------------------------------
        # GPS recovery failed
        # ----------------------------------------------------------

        if (
            event
            == "GPS_RECOVERY_FAILED"
        ):

            if self.gps_recovery_active:

                self.gps_recovery_result = (
                    "GPS_RECOVERY_FAILED"
                )

            return

        # ----------------------------------------------------------
        # Other scenarios intentionally unsupported in this stage
        # ----------------------------------------------------------

        self.get_logger().warning(
            f"Event not supported in "
            f"GPS-only BT stage: "
            f"{event}"
        )

    # ==============================================================
    # Tree execution
    # ==============================================================

    def tick_tree(
        self,
    ):

        if self.finished:
            return

        self.tree.tick()

        # ----------------------------------------------------------
        # Safe termination caused by GPS failure
        # ----------------------------------------------------------

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
                f"Reason: "
                f"{self.termination_reason}"
            )

            self.get_logger().info(
                "========================================"
            )

            self.timer.cancel()

            return

        status = (
            self.tree.root.status
        )

        # ----------------------------------------------------------
        # Normal mission completion
        # ----------------------------------------------------------

        if (
            status
            == py_trees.common.Status.SUCCESS
        ):

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

        elif (
            status
            == py_trees.common.Status.FAILURE
        ):

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

        self.current_task = (
            task
        )

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

        self.current_task = (
            task
        )

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

    def on_gps_recovered(
        self,
    ):

        if self.current_task is None:
            return

        self.get_logger().info(
            f"GPS RECOVERED -> "
            f"resume "
            f"{self.node_name(self.current_task)}"
        )

    def on_task_completed(
        self,
        task,
    ):

        self.completed_tasks += 1

        if (
            task["type"]
            == "navigate"
        ):

            self.completed_navigation_tasks += (
                1
            )

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
    # Safe termination
    # ==============================================================

    def mark_safe_terminated(
        self,
        reason,
    ):

        self.safe_terminated = True

        self.termination_reason = (
            reason
        )

        self.current_task = None

    # ==============================================================
    # Names / action descriptions
    # ==============================================================

    @staticmethod
    def node_name(
        task,
    ):

        if (
            task["type"]
            == "navigate"
        ):

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
    def action_description(
        task,
    ):

        task_type = (
            task["type"]
        )

        if task_type == "takeoff":
            return "TAKEOFF"

        if task_type == "navigate":

            return (
                f"NAVIGATE -> "
                f"{task['target']}"
            )

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
    # Progress interface
    # ==============================================================

    def publish_task_progress(
        self,
        task,
        status,
    ):

        payload = {

            "mission":
                self.mission_id,

            "formalism":
                "BT",

            "task_index":
                task["index"],

            "task_id":
                task["id"],

            "task_name":
                self.node_name(task),

            "task_type":
                task["type"],

            "task_status":
                status,

            "target":
                task["target"],

            "completed_task_count":
                self.completed_tasks,

            "completed_navigation_count":
                self.completed_navigation_tasks,
        }

        msg = String()

        msg.data = (
            json.dumps(
                payload,
                separators=(",", ":"),
            )
        )

        self.progress_pub.publish(
            msg
        )

    def publish_mission_status(
        self,
        status,
    ):

        payload = {

            "mission":
                self.mission_id,

            "formalism":
                "BT",

            "mission_status":
                status,

            "completed_task_count":
                self.completed_tasks,

            "completed_navigation_count":
                self.completed_navigation_tasks,

            "termination_reason":
                self.termination_reason,
        }

        msg = String()

        msg.data = (
            json.dumps(
                payload,
                separators=(",", ":"),
            )
        )

        self.progress_pub.publish(
            msg
        )

    # ==============================================================
    # Structural information
    # ==============================================================

    def print_tree_structure(
        self,
    ):

        nodes = list(
            self.tree.root.iterate()
        )

        total_nodes = (
            len(nodes)
        )

        leaf_nodes = sum(

            1

            for node in nodes

            if not node.children
        )

        composite_nodes = (
            total_nodes
            - leaf_nodes
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "BEHAVIOR TREE STRUCTURE"
        )

        self.get_logger().info(
            f"BT total nodes: "
            f"{total_nodes}"
        )

        self.get_logger().info(
            f"BT composite nodes: "
            f"{composite_nodes}"
        )

        self.get_logger().info(
            f"BT leaf/action nodes: "
            f"{leaf_nodes}"
        )

        self.get_logger().info(
            "Root: GPS_CAPABLE_ROOT"
        )

        self.get_logger().info(
            "Root type: Parallel"
        )

        self.get_logger().info(
            "GPS recovery nodes: 1 shared controller"
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(

            "\n"
            + py_trees.display.unicode_tree(
                self.tree.root,
                show_status=False,
            )
        )


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = (
        BehaviorTreeExecutor()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
