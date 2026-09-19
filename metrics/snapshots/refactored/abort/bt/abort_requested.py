"""Abort-request Behavior Tree controller."""

import time

import py_trees


class AbortRequestController(
    py_trees.behaviour.Behaviour
):
    """Return home after a mission abort request."""

    def __init__(
        self,
        name,
        executor,
        rtl_duration,
    ):
        super().__init__(name=name)

        self.executor = executor
        self.rtl_duration = float(
            rtl_duration
        )

        self.rtl_started = False
        self.rtl_start_time = None

    def update(self):
        """Execute abort-request response."""

        if self.executor.safe_terminated:
            return py_trees.common.Status.SUCCESS

        if not self.executor.abort_requested_active:
            self._reset_rtl()
            return py_trees.common.Status.SUCCESS

        return self._handle_return_to_home()

    def _handle_return_to_home(self):
        """Run return-to-home action."""

        if not self.rtl_started:
            self.rtl_started = True
            self.rtl_start_time = time.monotonic()

            self.executor.abort_rtl_active = True

            self.executor.get_logger().warning(
                'BT ABORT REQUEST HANDLER ACTIVE'
            )

            self.executor.get_logger().warning(
                'ACTION: RETURN_TO_HOME'
            )

        elapsed = (
            time.monotonic()
            - self.rtl_start_time
        )

        if elapsed < self.rtl_duration:
            return py_trees.common.Status.RUNNING

        self.executor.get_logger().info(
            'BT RETURN_TO_HOME COMPLETED'
        )

        self.executor.abort_rtl_active = False
        self.executor.abort_requested_active = False

        self.executor.mark_safe_terminated(
            'ABORT_REQUESTED'
        )

        return py_trees.common.Status.SUCCESS

    def _reset_rtl(self) -> None:
        """Reset return-to-home state."""

        self.rtl_started = False
        self.rtl_start_time = None
