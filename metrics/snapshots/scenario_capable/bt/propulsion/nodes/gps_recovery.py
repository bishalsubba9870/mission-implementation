#!/usr/bin/env python3

import time

import py_trees


class GPSRecoveryController(py_trees.behaviour.Behaviour):
    """
    Shared GPS recovery behaviour.

    Normal condition:
        SUCCESS

    GPS lost:
        RUNNING while waiting for recovery result.

    GPS available:
        SUCCESS and mission execution resumes.

    GPS recovery failed:
        perform emergency landing and mark the mission safely terminated.

    This node is shared by the entire mission. It is not duplicated
    for every navigation waypoint.
    """

    def __init__(
        self,
        name,
        executor,
        landing_duration,
    ):
        super().__init__(name=name)

        self.executor = executor

        self.landing_duration = (
            landing_duration
        )

        self.landing_start_time = None

        self.landing_started = False

    def initialise(self):
        pass

    def update(self):

        # ----------------------------------------------------------
        # Mission already safely terminated
        # ----------------------------------------------------------

        if self.executor.safe_terminated:
            return py_trees.common.Status.SUCCESS

        # ----------------------------------------------------------
        # Emergency landing following failed GPS recovery
        # ----------------------------------------------------------

        if self.executor.gps_emergency_landing:

            if not self.landing_started:

                self.landing_started = True

                self.landing_start_time = (
                    time.monotonic()
                )

                self.executor.get_logger().warning(
                    "GPS recovery failed."
                )

                self.executor.get_logger().info(
                    "BT GPS RECOVERY: "
                    "EMERGENCY LANDING STARTED"
                )

                self.executor.get_logger().info(
                    "ACTION: LAND"
                )

            elapsed = (
                time.monotonic()
                - self.landing_start_time
            )

            if elapsed < self.landing_duration:

                return (
                    py_trees.common.Status.RUNNING
                )

            self.executor.get_logger().info(
                "BT GPS RECOVERY: "
                "EMERGENCY LANDING COMPLETED"
            )

            self.executor.mark_safe_terminated(
                "GPS_RECOVERY_FAILED"
            )

            return py_trees.common.Status.SUCCESS

        # ----------------------------------------------------------
        # GPS recovery currently active
        # ----------------------------------------------------------

        if self.executor.gps_recovery_active:

            result = (
                self.executor.gps_recovery_result
            )

            # No follow-up result yet.
            if result is None:

                return (
                    py_trees.common.Status.RUNNING
                )

            # ------------------------------------------------------
            # Recovery successful
            # ------------------------------------------------------

            if result == "GPS_AVAILABLE":

                self.executor.get_logger().info(
                    "GPS recovery successful."
                )

                self.executor.get_logger().info(
                    "BT GPS RECOVERY: "
                    "GPS_AVAILABLE"
                )

                self.executor.gps_recovery_active = (
                    False
                )

                self.executor.gps_recovery_result = (
                    None
                )

                self.executor.on_gps_recovered()

                return (
                    py_trees.common.Status.SUCCESS
                )

            # ------------------------------------------------------
            # Recovery failed
            # ------------------------------------------------------

            if (
                result
                == "GPS_RECOVERY_FAILED"
            ):

                self.executor.gps_recovery_active = (
                    False
                )

                self.executor.gps_recovery_result = (
                    None
                )

                self.executor.gps_emergency_landing = (
                    True
                )

                # Landing begins on the next tick.
                return (
                    py_trees.common.Status.RUNNING
                )

        # ----------------------------------------------------------
        # Normal condition
        # ----------------------------------------------------------

        return py_trees.common.Status.SUCCESS

    def terminate(
        self,
        new_status,
    ):
        pass
