#!/usr/bin/env python3

import time

import py_trees


class WindUnsafeController(py_trees.behaviour.Behaviour):
    """
    Shared Behavior Tree handler for unsafe wind.

    Required experiment semantics:

        WIND_UNSAFE
            -> ABORT_MISSION
            -> SAFE_LAND
            -> SAFE_TERMINATED

    The handler is shared across the complete mission rather than
    duplicated inside every mission-action node.
    """

    def __init__(
        self,
        name,
        executor,
        abort_duration,
        landing_duration,
    ):
        super().__init__(
            name=name
        )

        self.executor = executor

        self.abort_duration = float(
            abort_duration
        )

        self.landing_duration = float(
            landing_duration
        )

        self.phase = "IDLE"

        self.phase_start_time = None

    def initialise(self):
        pass

    def update(self):

        # ==========================================================
        # No unsafe-wind condition
        # ==========================================================

        if not self.executor.wind_unsafe_active:

            self.phase = "IDLE"

            self.phase_start_time = None

            return py_trees.common.Status.SUCCESS

        now = time.monotonic()

        # ==========================================================
        # Phase 1 — ABORT_MISSION
        # ==========================================================

        if self.phase == "IDLE":

            self.phase = "ABORT"

            self.phase_start_time = now

            self.executor.get_logger().warning(
                "BT WIND UNSAFE HANDLER ACTIVE"
            )

            self.executor.get_logger().warning(
                "ACTION: ABORT_MISSION"
            )

            return py_trees.common.Status.RUNNING

        if self.phase == "ABORT":

            elapsed = (
                now
                - self.phase_start_time
            )

            if elapsed < self.abort_duration:

                return py_trees.common.Status.RUNNING

            self.executor.get_logger().info(
                "BT ABORT_MISSION COMPLETED"
            )

            # ======================================================
            # Phase 2 — SAFE_LAND
            # ======================================================

            self.phase = "SAFE_LAND"

            self.phase_start_time = now

            self.executor.wind_safe_landing_active = True

            self.executor.get_logger().warning(
                "ACTION: SAFE_LAND"
            )

            return py_trees.common.Status.RUNNING

        # ==========================================================
        # SAFE_LAND executing
        # ==========================================================

        if self.phase == "SAFE_LAND":

            elapsed = (
                now
                - self.phase_start_time
            )

            if elapsed < self.landing_duration:

                return py_trees.common.Status.RUNNING

            self.executor.get_logger().info(
                "BT SAFE_LAND COMPLETED"
            )

            self.executor.wind_safe_landing_active = False

            self.executor.wind_unsafe_active = False

            self.phase = "COMPLETED"

            self.executor.mark_safe_terminated(
                "WIND_UNSAFE"
            )

            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.SUCCESS
