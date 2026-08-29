#!/usr/bin/env python3

import time

import py_trees


class BatteryCriticalController(py_trees.behaviour.Behaviour):
    """
    Shared BT handler for critical battery.

    BATTERY_CRITICAL
        -> EMERGENCY_LAND
        -> SAFE_TERMINATED
    """

    def __init__(
        self,
        name,
        executor,
        landing_duration,
    ):
        super().__init__(name=name)

        self.executor = executor
        self.landing_duration = float(landing_duration)

        self.landing_started = False
        self.landing_start_time = None

    def initialise(self):
        pass

    def update(self):

        if not self.executor.battery_critical_active:
            self.landing_started = False
            self.landing_start_time = None

            return py_trees.common.Status.SUCCESS

        if not self.landing_started:
            self.landing_started = True
            self.landing_start_time = time.monotonic()

            self.executor.get_logger().warning(
                "BT BATTERY CRITICAL HANDLER ACTIVE"
            )

            self.executor.get_logger().warning(
                "ACTION: EMERGENCY_LAND"
            )

            return py_trees.common.Status.RUNNING

        elapsed = (
            time.monotonic()
            - self.landing_start_time
        )

        if elapsed < self.landing_duration:
            return py_trees.common.Status.RUNNING

        self.executor.get_logger().info(
            "BT EMERGENCY LANDING COMPLETED"
        )

        self.executor.battery_critical_active = False

        self.executor.mark_safe_terminated(
            "BATTERY_CRITICAL"
        )

        return py_trees.common.Status.SUCCESS
