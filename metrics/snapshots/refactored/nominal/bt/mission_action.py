"""Nominal Behavior Tree mission-action leaf."""

import time

import py_trees


class MissionAction(
    py_trees.behaviour.Behaviour
):
    """Execute one nominal mission task."""

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

        self.start_time = None
        self.completion_reported = False

    def initialise(self):
        """Start task execution."""

        self.start_time = time.monotonic()
        self.completion_reported = False

        self.executor.on_task_started(
            self.task
        )

    def update(self):
        """Return task execution status."""

        elapsed = (
            time.monotonic()
            - self.start_time
        )

        if elapsed < self.duration:
            return (
                py_trees.common.Status.RUNNING
            )

        if not self.completion_reported:
            self.completion_reported = True

            self.executor.on_task_completed(
                self.task
            )

        return py_trees.common.Status.SUCCESS

    def terminate(
        self,
        new_status,
    ):
        """Reset timing when the leaf terminates."""

        self.start_time = None
