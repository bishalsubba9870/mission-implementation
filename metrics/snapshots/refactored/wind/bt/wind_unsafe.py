"""Unsafe-wind Behavior Tree controller."""

import time

import py_trees


class WindUnsafeController(
    py_trees.behaviour.Behaviour
):
    """Abort mission and safely land after unsafe wind."""

    def __init__(
        self,
        name,
        executor,
        abort_duration,
        landing_duration,
    ):
        super().__init__(name=name)

        self.executor = executor
        self.abort_duration = float(abort_duration)
        self.landing_duration = float(landing_duration)

        self.phase = 'IDLE'
        self.phase_start_time = None

    def update(self):
        """Execute unsafe-wind response."""

        if self.executor.safe_terminated:
            return py_trees.common.Status.SUCCESS

        if not self.executor.wind_unsafe_active:
            self._reset()
            return py_trees.common.Status.SUCCESS

        now = time.monotonic()

        if self.phase == 'IDLE':
            return self._start_abort(now)

        if self.phase == 'ABORT':
            return self._update_abort(now)

        if self.phase == 'SAFE_LAND':
            return self._update_safe_land(now)

        return py_trees.common.Status.SUCCESS

    def _start_abort(
        self,
        now,
    ):
        """Start mission abort."""

        self.phase = 'ABORT'
        self.phase_start_time = now

        self.executor.get_logger().warning(
            'BT WIND UNSAFE HANDLER ACTIVE'
        )

        self.executor.get_logger().warning(
            'ACTION: ABORT_MISSION'
        )

        return py_trees.common.Status.RUNNING

    def _update_abort(
        self,
        now,
    ):
        """Complete abort phase."""

        elapsed = (
            now
            - self.phase_start_time
        )

        if elapsed < self.abort_duration:
            return py_trees.common.Status.RUNNING

        self.executor.get_logger().info(
            'BT ABORT_MISSION COMPLETED'
        )

        self.phase = 'SAFE_LAND'
        self.phase_start_time = now

        self.executor.wind_safe_landing_active = True

        self.executor.get_logger().warning(
            'ACTION: SAFE_LAND'
        )

        return py_trees.common.Status.RUNNING

    def _update_safe_land(
        self,
        now,
    ):
        """Complete safe landing."""

        elapsed = (
            now
            - self.phase_start_time
        )

        if elapsed < self.landing_duration:
            return py_trees.common.Status.RUNNING

        self.executor.get_logger().info(
            'BT SAFE_LAND COMPLETED'
        )

        self.executor.wind_safe_landing_active = False
        self.executor.wind_unsafe_active = False

        self.phase = 'COMPLETED'

        self.executor.mark_safe_terminated(
            'WIND_UNSAFE'
        )

        return py_trees.common.Status.SUCCESS

    def _reset(self) -> None:
        """Reset controller state."""

        if self.phase == 'COMPLETED':
            return

        self.phase = 'IDLE'
        self.phase_start_time = None
