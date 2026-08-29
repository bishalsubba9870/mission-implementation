#!/usr/bin/env python3

import time

import py_trees


class MissionAction(py_trees.behaviour.Behaviour):
    """
    Mission action leaf.

    The node remains RUNNING for a configured logical duration
    and then returns SUCCESS.

    During GPS recovery, a running navigation action is paused.
    After GPS_AVAILABLE it continues the same action rather than
    advancing to another mission task.
    """

    def __init__(
        self,
        name,
        task,
        executor,
        duration,
    ):
        super().__init__(
            name=name
        )

        self.task = task

        self.executor = executor

        self.duration = float(
            duration
        )

        self.remaining_duration = (
            float(duration)
        )

        self.last_tick_time = None

        self.started = False

        self.completed = False

        self.pause_reported = False

    def initialise(self):

        self.last_tick_time = (
            time.monotonic()
        )

        if not self.started:

            self.started = True

            self.executor.on_task_started(
                self.task
            )

        else:

            self.executor.on_task_resumed(
                self.task
            )

    def update(self):

        # ----------------------------------------------------------
        # Mission has been terminated
        # ----------------------------------------------------------

        if self.executor.safe_terminated:

            return (
                py_trees.common.Status.RUNNING
            )

        # ----------------------------------------------------------
        # Emergency landing takes priority
        # ----------------------------------------------------------

        if self.executor.gps_emergency_landing:

            return (
                py_trees.common.Status.RUNNING
            )

        # ----------------------------------------------------------
        # GPS affects navigation actions only
        # ----------------------------------------------------------

        if (
            self.task["type"] == "navigate"
            and self.executor.gps_recovery_active
        ):

            if not self.pause_reported:

                self.pause_reported = True

                self.executor.on_task_paused(
                    self.task,
                    reason="GPS_LOST",
                )

            # Reset timing reference so recovery time is not
            # counted as navigation execution time.
            self.last_tick_time = (
                time.monotonic()
            )

            return (
                py_trees.common.Status.RUNNING
            )

        # ----------------------------------------------------------
        # Recovery completed: continue same navigation action
        # ----------------------------------------------------------

        if self.pause_reported:

            self.pause_reported = False

            self.executor.get_logger().info(
                f"BT NODE RESUMING: "
                f"{self.executor.node_name(self.task)}"
            )

        # ----------------------------------------------------------
        # Logical execution timing
        # ----------------------------------------------------------

        now = time.monotonic()

        if self.last_tick_time is None:

            self.last_tick_time = now

        elapsed = (
            now
            - self.last_tick_time
        )

        self.last_tick_time = now

        self.remaining_duration -= (
            elapsed
        )

        if self.remaining_duration > 0.0:

            return (
                py_trees.common.Status.RUNNING
            )

        # ----------------------------------------------------------
        # Task completed
        # ----------------------------------------------------------

        if not self.completed:

            self.completed = True

            self.remaining_duration = 0.0

            self.executor.on_task_completed(
                self.task
            )

        return py_trees.common.Status.SUCCESS

    def terminate(
        self,
        new_status,
    ):

        self.last_tick_time = None
