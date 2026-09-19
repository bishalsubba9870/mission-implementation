"""Shared GPS recovery Behavior Tree node."""

import time

import py_trees


class GPSRecoveryController(
    py_trees.behaviour.Behaviour
):
    """Handle GPS recovery for the entire mission."""

    def __init__(
        self,
        name,
        executor,
        landing_duration,
    ):
        super().__init__(name=name)

        self.executor = executor
        self.landing_duration = float(
            landing_duration
        )

        self.landing_started = False
        self.landing_start_time = None

    def update(self):
        """Return current GPS recovery status."""

        if self.executor.safe_terminated:
            return py_trees.common.Status.SUCCESS

        if self.executor.gps_emergency_landing:
            return self._handle_emergency_landing()

        if self.executor.gps_recovery_active:
            return self._handle_recovery()

        return py_trees.common.Status.SUCCESS

    def _handle_recovery(self):
        """Process GPS recovery result."""

        result = self.executor.gps_recovery_result

        if result is None:
            return py_trees.common.Status.RUNNING

        if result == 'GPS_AVAILABLE':
            self.executor.gps_recovery_active = False
            self.executor.gps_recovery_result = None

            self.executor.on_gps_recovered()

            return py_trees.common.Status.SUCCESS

        if result == 'GPS_RECOVERY_FAILED':
            self.executor.gps_recovery_active = False
            self.executor.gps_recovery_result = None
            self.executor.gps_emergency_landing = True

            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def _handle_emergency_landing(self):
        """Perform emergency landing after failed recovery."""

        if not self.landing_started:
            self.landing_started = True
            self.landing_start_time = time.monotonic()

            self.executor.get_logger().warning(
                'GPS recovery failed.'
            )

            self.executor.get_logger().info(
                'BT GPS RECOVERY: EMERGENCY LANDING STARTED'
            )

            self.executor.get_logger().info(
                'ACTION: LAND'
            )

        elapsed = (
            time.monotonic()
            - self.landing_start_time
        )

        if elapsed < self.landing_duration:
            return py_trees.common.Status.RUNNING

        self.executor.get_logger().info(
            'BT GPS RECOVERY: EMERGENCY LANDING COMPLETED'
        )

        self.executor.mark_safe_terminated(
            'GPS_RECOVERY_FAILED'
        )

        return py_trees.common.Status.SUCCESS
