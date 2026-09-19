"""Critical-battery Behavior Tree controller."""

import time

import py_trees


class BatteryCriticalController(
    py_trees.behaviour.Behaviour
):
    """Perform emergency landing after critical battery."""

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
        """Execute critical-battery response."""

        if self.executor.safe_terminated:
            return py_trees.common.Status.SUCCESS

        if not self.executor.battery_critical_active:
            self._reset_landing()
            return py_trees.common.Status.SUCCESS

        return self._handle_emergency_landing()

    def _handle_emergency_landing(self):
        """Run emergency landing."""

        if not self.landing_started:
            self.landing_started = True
            self.landing_start_time = time.monotonic()

            self.executor.get_logger().warning(
                'BT BATTERY CRITICAL HANDLER ACTIVE'
            )

            self.executor.get_logger().warning(
                'ACTION: EMERGENCY_LAND'
            )

        elapsed = (
            time.monotonic()
            - self.landing_start_time
        )

        if elapsed < self.landing_duration:
            return py_trees.common.Status.RUNNING

        self.executor.get_logger().info(
            'BT EMERGENCY LANDING COMPLETED'
        )

        self.executor.battery_critical_active = False

        self.executor.mark_safe_terminated(
            'BATTERY_CRITICAL'
        )

        return py_trees.common.Status.SUCCESS

    def _reset_landing(self) -> None:
        """Reset emergency-landing state."""

        self.landing_started = False
        self.landing_start_time = None
