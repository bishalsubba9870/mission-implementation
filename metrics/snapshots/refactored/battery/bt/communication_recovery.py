"""Shared communication-recovery Behavior Tree controller."""

import time

import py_trees


class CommunicationRecoveryController(
    py_trees.behaviour.Behaviour
):
    """Recover communication or return home after recovery failure."""

    def __init__(
        self,
        name,
        executor,
        rtl_duration,
    ):
        super().__init__(name=name)

        self.executor = executor
        self.rtl_duration = float(rtl_duration)

        self.rtl_started = False
        self.rtl_start_time = None

    def update(self):
        """Return communication recovery status."""

        if self.executor.safe_terminated:
            return py_trees.common.Status.SUCCESS

        if not self.executor.communication_loss_active:
            self._reset_rtl()
            return py_trees.common.Status.SUCCESS

        result = (
            self.executor.communication_recovery_result
        )

        if result is None:
            return py_trees.common.Status.RUNNING

        if result == 'COMMUNICATION_AVAILABLE':
            return self._recover_communication()

        if result == 'COMMUNICATION_RECOVERY_FAILED':
            return self._handle_failed_recovery()

        return py_trees.common.Status.RUNNING

    def _recover_communication(self):
        """Complete successful communication recovery."""

        self.executor.communication_loss_active = False
        self.executor.communication_recovery_result = None

        self.executor.on_communication_recovered()

        return py_trees.common.Status.SUCCESS

    def _handle_failed_recovery(self):
        """Return home after failed communication recovery."""

        if not self.rtl_started:
            self.rtl_started = True
            self.rtl_start_time = time.monotonic()

            self.executor.communication_rtl_active = True

            self.executor.get_logger().warning(
                'BT COMMUNICATION RECOVERY FAILED'
            )

            self.executor.get_logger().warning(
                'ACTION: RETURN_TO_HOME'
            )

            return py_trees.common.Status.RUNNING

        elapsed = (
            time.monotonic()
            - self.rtl_start_time
        )

        if elapsed < self.rtl_duration:
            return py_trees.common.Status.RUNNING

        self.executor.get_logger().info(
            'BT RETURN_TO_HOME COMPLETED'
        )

        self.executor.communication_rtl_active = False
        self.executor.communication_loss_active = False
        self.executor.communication_recovery_result = None

        self.executor.mark_safe_terminated(
            'COMMUNICATION_RECOVERY_FAILED'
        )

        return py_trees.common.Status.SUCCESS

    def _reset_rtl(self) -> None:
        """Reset return-to-home timing."""

        self.rtl_started = False
        self.rtl_start_time = None
