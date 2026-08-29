#!/usr/bin/env python3

import time

import py_trees


class PropulsionFailureController(py_trees.behaviour.Behaviour):
    """
    Shared Behavior Tree controller for propulsion failure.

    Runtime behaviour:

        normal mission
            ↓
        PROPULSION_FAILURE
            ↓
        emergency landing
            ↓
        SAFE_TERMINATED

    The controller is shared by the complete mission and is not
    duplicated for every navigation waypoint.
    """

    def __init__(
        self,
        name,
        executor,
        landing_duration,
    ):
        super().__init__(
            name=name
        )

        self.executor = executor

        self.landing_duration = float(
            landing_duration
        )

        self.landing_started = False

        self.landing_start_time = None

    def initialise(self):
        """
        Called whenever this behaviour enters RUNNING from another
        status.

        Runtime state is maintained explicitly so the emergency
        landing can continue over multiple BT ticks.
        """
        pass

    def update(self):
        """
        Handle propulsion-failure emergency landing.
        """

        # ----------------------------------------------------------
        # No propulsion failure
        # ----------------------------------------------------------

        if not self.executor.propulsion_failure_active:
            self.landing_started = False
            self.landing_start_time = None

            return py_trees.common.Status.SUCCESS

        # ----------------------------------------------------------
        # Start emergency landing
        # ----------------------------------------------------------

        if not self.landing_started:
            self.landing_started = True

            self.landing_start_time = (
                time.monotonic()
            )

            self.executor.get_logger().warning(
                "BT PROPULSION FAILURE HANDLER ACTIVE"
            )

            self.executor.get_logger().warning(
                "ACTION: EMERGENCY_LAND"
            )

            return py_trees.common.Status.RUNNING

        # ----------------------------------------------------------
        # Emergency landing still running
        # ----------------------------------------------------------

        elapsed = (
            time.monotonic()
            - self.landing_start_time
        )

        if elapsed < self.landing_duration:
            return py_trees.common.Status.RUNNING

        # ----------------------------------------------------------
        # Emergency landing completed
        # ----------------------------------------------------------

        self.executor.get_logger().info(
            "BT EMERGENCY LANDING COMPLETED"
        )

        self.executor.propulsion_failure_active = False

        self.executor.mark_safe_terminated(
            "PROPULSION_FAILURE"
        )

        return py_trees.common.Status.SUCCESS
