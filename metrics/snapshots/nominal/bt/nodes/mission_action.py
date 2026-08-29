#!/usr/bin/env python3

import time
import py_trees


class MissionAction(py_trees.behaviour.Behaviour):
    """
    Generic nominal mission action leaf.

    Each action remains RUNNING for a configurable logical duration
    and then returns SUCCESS.

    This represents mission-logic execution only.
    It does not represent physical UAV flight time.
    """

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
        self.duration = duration

        self.start_time = None
        self.completion_reported = False

    def initialise(self):
        self.start_time = time.monotonic()
        self.completion_reported = False

        self.executor.on_task_started(
            self.task
        )

    def update(self):
        elapsed = (
            time.monotonic()
            - self.start_time
        )

        if elapsed < self.duration:
            return py_trees.common.Status.RUNNING

        if not self.completion_reported:
            self.completion_reported = True

            self.executor.on_task_completed(
                self.task
            )

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        pass
