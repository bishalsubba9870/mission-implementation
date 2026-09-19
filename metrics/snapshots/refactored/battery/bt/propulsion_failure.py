"""Shared propulsion-failure Behavior Tree controller."""

import time

import py_trees


class PropulsionFailureController(
    py_trees.behaviour.Behaviour
):
    """Perform emergency landing after propulsion failure."""

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
        """Return propulsion recovery status."""

        if self.executor.safe_terminated:
            return py_trees.common.Status.SUCCESS

        if not self.executor.propulsion_failure_active:
            self._reset_landing()

            return py_trees.common.Status.SUCCESS

        return self._handle_emergency_landing()

    def _handle_emergency_landing(self):
        """Run propulsion emergency landing."""

        if not self.landing_started:
            self.landing_started = True
            self.landing_start_time = time.monotonic()

            self.executor.get_logger().error(
                'PROPULSION FAILURE DETECTED'
            )

            self.executor.get_logger().warning(
                'BT PROPULSION FAILURE HANDLER ACTIVE'
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

        self.executor.propulsion_failure_active = False

        self.executor.mark_safe_terminated(
            'PROPULSION_FAILURE'
        )

        return py_trees.common.Status.SUCCESS

    def _reset_landing(self) -> None:
        """Reset controller runtime state."""

        self.landing_started = False
        self.landing_start_time = None
