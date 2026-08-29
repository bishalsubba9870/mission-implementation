#!/usr/bin/env python3

import time

import py_trees


class AbortRequestedController(py_trees.behaviour.Behaviour):
    """
    Shared BT handler for operator/user requested mission abort.

    Required semantics:

        ABORT_REQUESTED
            -> ABORT_MISSION
            -> RETURN_TO_HOME
            -> SAFE_TERMINATED
    """

    def __init__(
        self,
        name,
        executor,
        abort_duration,
        rtl_duration,
    ):
        super().__init__(name=name)

        self.executor = executor

        self.abort_duration = float(abort_duration)
        self.rtl_duration = float(rtl_duration)

        self.phase = "IDLE"
        self.phase_start_time = None

    def initialise(self):
        pass

    def update(self):

        if not self.executor.abort_requested_active:

            self.phase = "IDLE"
            self.phase_start_time = None

            return py_trees.common.Status.SUCCESS

        now = time.monotonic()

        # ==========================================================
        # Phase 1: ABORT_MISSION
        # ==========================================================

        if self.phase == "IDLE":

            self.phase = "ABORT"

            self.phase_start_time = now

            self.executor.get_logger().warning(
                "BT ABORT REQUEST HANDLER ACTIVE"
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
            # Phase 2: RETURN_TO_HOME
            # ======================================================

            self.phase = "RTL"

            self.phase_start_time = now

            self.executor.abort_rtl_active = True

            self.executor.get_logger().warning(
                "ACTION: RETURN_TO_HOME"
            )

            return py_trees.common.Status.RUNNING

        # ==========================================================
        # RTL execution
        # ==========================================================

        if self.phase == "RTL":

            elapsed = (
                now
                - self.phase_start_time
            )

            if elapsed < self.rtl_duration:

                return py_trees.common.Status.RUNNING

            self.executor.get_logger().info(
                "BT RETURN_TO_HOME COMPLETED"
            )

            self.executor.abort_rtl_active = False
            self.executor.abort_requested_active = False

            self.phase = "COMPLETED"

            self.executor.mark_safe_terminated(
                "ABORT_REQUESTED"
            )

            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.SUCCESS
