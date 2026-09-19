"""Behavior Tree mission-action leaf with runtime pause/resume."""

import time

import py_trees


class MissionAction(
    py_trees.behaviour.Behaviour
):
    """Execute one mission task with recovery-aware pausing."""

    def __init__(
        self,
        name,
        task,
        executor,
        duration,
    ):
        super().__init__(name=name)

        self.task = task
        self.executor = executor
        self.duration = float(duration)

        self.remaining_duration = float(duration)
        self.last_tick_time = None

        self.started = False
        self.completed = False
        self.pause_reason = None

    def initialise(self):
        """Start or resume task execution."""

        self.last_tick_time = time.monotonic()

        if not self.started:
            self.started = True

            self.executor.on_task_started(
                self.task
            )
        else:
            self.executor.on_task_resumed(
                self.task
            )

    def update(self):
        """Return current task status."""

        if self.executor.safe_terminated:
            return py_trees.common.Status.RUNNING

        pause_reason = self._pause_reason()

        if pause_reason is not None:
            self._pause_task(
                pause_reason
            )

            return py_trees.common.Status.RUNNING

        self._report_resume()

        return self._update_execution_time()

    def _pause_reason(self):
        """Return active recovery reason, if any."""

        if self.executor.propulsion_failure_active:
            return 'PROPULSION_FAILURE'

        if self.executor.gps_emergency_landing:
            return 'GPS_EMERGENCY_LANDING'

        if (
            self.task['type'] == 'navigate'
            and self.executor.gps_recovery_active
        ):
            return 'GPS_LOST'

        return None

    def _pause_task(
        self,
        reason,
    ) -> None:
        """Pause task execution during recovery."""

        if self.pause_reason != reason:
            self.pause_reason = reason

            self.executor.on_task_paused(
                self.task,
                reason=reason,
            )

        self.last_tick_time = time.monotonic()

    def _report_resume(self) -> None:
        """Report task resumption once."""

        if self.pause_reason is None:
            return

        self.pause_reason = None

        self.executor.get_logger().info(
            f'BT NODE RESUMING: '
            f'{self.executor.node_name(self.task)}'
        )

    def _update_execution_time(self):
        """Advance logical task execution."""

        now = time.monotonic()

        if self.last_tick_time is None:
            self.last_tick_time = now

        elapsed = now - self.last_tick_time
        self.last_tick_time = now

        self.remaining_duration -= elapsed

        if self.remaining_duration > 0.0:
            return py_trees.common.Status.RUNNING

        if not self.completed:
            self.completed = True
            self.remaining_duration = 0.0

            self.executor.on_task_completed(
                self.task
            )

        return py_trees.common.Status.SUCCESS

    def terminate(
        self,
        new_status,
    ):
        """Reset tick timing."""

        self.last_tick_time = None
