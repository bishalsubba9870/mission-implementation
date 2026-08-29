#!/usr/bin/env python3

import time

import py_trees


class MissionAction(py_trees.behaviour.Behaviour):
    """
    Reusable mission-action leaf.

    Runtime priorities:

        1. safe termination
        2. propulsion failure
        3. unsafe wind
        4. communication loss
        5. GPS emergency landing
        6. GPS recovery
        7. normal mission execution

    Scenario handling pauses mission-action progress rather than
    allowing the mission sequence to continue in parallel.
    """

    def __init__(
        self,
        name,
        task,
        executor,
        task_duration,
    ):
        super().__init__(
            name=name
        )

        self.task = task

        self.executor = executor

        self.duration = float(
            task_duration
        )

        self.remaining_duration = float(
            task_duration
        )

        self.last_tick_time = None

        self.started = False

        self.completed = False

        self.gps_pause_reported = False

        self.propulsion_pause_reported = False

        self.communication_pause_reported = False

        self.wind_pause_reported = False

    def initialise(
        self,
    ):

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

    def update(
        self,
    ):

        # ==========================================================
        # Mission already terminated
        # ==========================================================

        if self.executor.safe_terminated:

            self.last_tick_time = (
                time.monotonic()
            )

            return py_trees.common.Status.RUNNING

        # ==========================================================
        # Propulsion failure
        # ==========================================================

        if self.executor.propulsion_failure_active:

            if not self.propulsion_pause_reported:

                self.propulsion_pause_reported = True

                self.executor.on_task_paused(
                    self.task,
                    reason="PROPULSION_FAILURE",
                )

            self.last_tick_time = (
                time.monotonic()
            )

            return py_trees.common.Status.RUNNING

        if self.propulsion_pause_reported:

            self.propulsion_pause_reported = False

        # ==========================================================
        # Unsafe wind
        # ==========================================================

        if self.executor.wind_unsafe_active:

            if not self.wind_pause_reported:

                self.wind_pause_reported = True

                self.executor.on_task_paused(
                    self.task,
                    reason="WIND_UNSAFE",
                )

            self.last_tick_time = (
                time.monotonic()
            )

            return py_trees.common.Status.RUNNING

        if self.wind_pause_reported:

            self.wind_pause_reported = False

        # ==========================================================
        # Communication loss
        # ==========================================================

        if self.executor.communication_loss_active:

            if not self.communication_pause_reported:

                self.communication_pause_reported = True

                self.executor.on_task_paused(
                    self.task,
                    reason="COMMUNICATION_LOST",
                )

            self.last_tick_time = (
                time.monotonic()
            )

            return py_trees.common.Status.RUNNING

        if self.communication_pause_reported:

            self.communication_pause_reported = False

            self.executor.get_logger().info(
                f"BT NODE RESUMING: "
                f"{self.executor.node_name(self.task)}"
            )

        # ==========================================================
        # GPS emergency landing
        # ==========================================================

        if self.executor.gps_emergency_landing:

            self.last_tick_time = (
                time.monotonic()
            )

            return py_trees.common.Status.RUNNING

        # ==========================================================
        # GPS recovery
        # ==========================================================

        if (
            self.task["type"] == "navigate"
            and self.executor.gps_recovery_active
        ):

            if not self.gps_pause_reported:

                self.gps_pause_reported = True

                self.executor.on_task_paused(
                    self.task,
                    reason="GPS_LOST",
                )

            self.last_tick_time = (
                time.monotonic()
            )

            return py_trees.common.Status.RUNNING

        if self.gps_pause_reported:

            self.gps_pause_reported = False

            self.executor.get_logger().info(
                f"BT NODE RESUMING: "
                f"{self.executor.node_name(self.task)}"
            )

        # ==========================================================
        # Normal mission execution
        # ==========================================================

        now = time.monotonic()

        if self.last_tick_time is None:

            self.last_tick_time = now

        elapsed = (
            now
            - self.last_tick_time
        )

        self.last_tick_time = now

        self.remaining_duration -= elapsed

        if self.remaining_duration > 0.0:

            return py_trees.common.Status.RUNNING

        # ==========================================================
        # Mission task completed
        # ==========================================================

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
