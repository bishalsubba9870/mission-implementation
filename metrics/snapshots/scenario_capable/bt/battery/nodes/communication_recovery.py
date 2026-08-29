#!/usr/bin/env python3

import time

import py_trees


class CommunicationRecoveryController(
    py_trees.behaviour.Behaviour
):
    """
    Shared communication-loss recovery behaviour.

    COMMUNICATION_LOST
        -> wait for recovery result

    COMMUNICATION_RESTORED
        -> resume interrupted mission task

    COMMUNICATION_RECOVERY_FAILED
        -> RETURN_TO_HOME
        -> SAFE_TERMINATED
    """

    def __init__(
        self,
        name,
        executor,
        rtl_duration,
    ):
        super().__init__(
            name=name
        )

        self.executor = executor

        self.rtl_duration = float(
            rtl_duration
        )

        self.rtl_started = False
        self.rtl_start_time = None

    def initialise(self):
        pass

    def update(self):

        # ==========================================================
        # Communication loss not active
        # ==========================================================

        if not self.executor.communication_loss_active:

            self.rtl_started = False
            self.rtl_start_time = None

            return py_trees.common.Status.SUCCESS

        # ==========================================================
        # Waiting for recovery result
        # ==========================================================

        if self.executor.communication_recovery_result is None:

            return py_trees.common.Status.RUNNING

        # ==========================================================
        # Communication restored
        # ==========================================================

        if (
            self.executor.communication_recovery_result
            == "COMMUNICATION_RESTORED"
        ):

            self.executor.get_logger().info(
                "BT COMMUNICATION RECOVERY: "
                "COMMUNICATION_RESTORED"
            )

            self.executor.communication_loss_active = False
            self.executor.communication_recovery_result = None

            self.executor.on_communication_recovered()

            return py_trees.common.Status.SUCCESS

        # ==========================================================
        # Communication recovery failed
        # ==========================================================

        if (
            self.executor.communication_recovery_result
            == "COMMUNICATION_RECOVERY_FAILED"
        ):

            # ------------------------------------------------------
            # Start RTL
            # ------------------------------------------------------

            if not self.rtl_started:

                self.rtl_started = True

                self.rtl_start_time = (
                    time.monotonic()
                )

                self.executor.communication_rtl_active = True

                self.executor.get_logger().warning(
                    "BT COMMUNICATION RECOVERY FAILED"
                )

                self.executor.get_logger().warning(
                    "ACTION: RETURN_TO_HOME"
                )

                return py_trees.common.Status.RUNNING

            # ------------------------------------------------------
            # RTL executing
            # ------------------------------------------------------

            elapsed = (
                time.monotonic()
                - self.rtl_start_time
            )

            if elapsed < self.rtl_duration:

                return py_trees.common.Status.RUNNING

            # ------------------------------------------------------
            # RTL completed
            # ------------------------------------------------------

            self.executor.get_logger().info(
                "BT RETURN_TO_HOME COMPLETED"
            )

            self.executor.communication_rtl_active = False
            self.executor.communication_loss_active = False

            self.executor.mark_safe_terminated(
                "COMMUNICATION_RECOVERY_FAILED"
            )

            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
