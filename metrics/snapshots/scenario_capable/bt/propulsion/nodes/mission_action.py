#!/usr/bin/env python3

import time

import py_trees


class MissionAction(py_trees.behaviour.Behaviour):
    """
    Reusable Behavior Tree mission-action leaf.

    Current capability stage:

        - nominal mission execution
        - GPS recovery
        - propulsion-failure emergency handling

    Normal behaviour:
        task RUNNING
            ↓
        configured logical duration
            ↓
        SUCCESS

    GPS_LOST during navigation:
        navigation RUNNING
            ↓
        GPS recovery active
            ↓
        same navigation node PAUSED
            ↓
        GPS_AVAILABLE
            ↓
        same navigation node RESUMES

    PROPULSION_FAILURE:
        any running mission task
            ↓
        mission execution PAUSED
            ↓
        shared propulsion-failure controller performs
        EMERGENCY_LAND
            ↓
        SAFE_TERMINATED

    Recovery/emergency time is not counted as normal mission-action
    execution time.
    """

    def __init__(
        self,
        name,
        task,
        executor,
        task_duration,
    ):
        super().__init__(
            name=name
        )

        self.task = task

        self.executor = executor

        self.duration = float(
            task_duration
        )

        self.remaining_duration = float(
            task_duration
        )

        self.last_tick_time = None

        self.started = False

        self.completed = False

        # ----------------------------------------------------------
        # Pause-report flags
        # ----------------------------------------------------------

        self.gps_pause_reported = False

        self.propulsion_pause_reported = False

    # ==============================================================
    # Behaviour initialisation
    # ==============================================================

    def initialise(
        self,
    ):
        """
        Called whenever this behaviour enters RUNNING execution.

        For a newly reached mission action:
            on_task_started()

        For a previously interrupted/re-entered mission action:
            on_task_resumed()
        """

        self.last_tick_time = (
            time.monotonic()
        )

        if not self.started:

            self.started = True

            self.executor.on_task_started(
                self.task
            )

        else:

            self.executor.on_task_resumed(
                self.task
            )

    # ==============================================================
    # Main behaviour execution
    # ==============================================================

    def update(
        self,
    ):
        # ----------------------------------------------------------
        # Mission has already reached safe termination
        # ----------------------------------------------------------
        #
        # Keep this mission leaf RUNNING.
        #
        # The executor detects safe_terminated after the tree tick
        # and stops the complete mission executor.
        # ----------------------------------------------------------

        if self.executor.safe_terminated:

            self.last_tick_time = (
                time.monotonic()
            )

            return (
                py_trees.common.Status.RUNNING
            )

        # ==========================================================
        # PRIORITY 1:
        # Propulsion failure
        # ==========================================================
        #
        # Propulsion failure affects the complete UAV mission,
        # not only navigation.
        #
        # Therefore any currently running mission action must stop
        # progressing while the shared propulsion emergency handler
        # performs EMERGENCY_LAND.
        # ==========================================================

        if self.executor.propulsion_failure_active:

            if not self.propulsion_pause_reported:

                self.propulsion_pause_reported = True

                self.executor.on_task_paused(
                    self.task,
                    reason="PROPULSION_FAILURE",
                )

            # Do not count emergency-handling time as mission-action
            # execution time.
            self.last_tick_time = (
                time.monotonic()
            )

            return (
                py_trees.common.Status.RUNNING
            )

        # ----------------------------------------------------------
        # Propulsion pause ended
        # ----------------------------------------------------------
        #
        # Normally propulsion failure ends in SAFE_TERMINATED, so
        # this branch should rarely be reached.
        #
        # It is retained to keep the reusable leaf internally
        # consistent if the safety policy changes later.
        # ----------------------------------------------------------

        if self.propulsion_pause_reported:

            self.propulsion_pause_reported = False

            self.executor.get_logger().info(
                f"BT NODE PROPULSION PAUSE CLEARED: "
                f"{self.executor.node_name(self.task)}"
            )

        # ==========================================================
        # PRIORITY 2:
        # GPS emergency landing
        # ==========================================================
        #
        # GPS_RECOVERY_FAILED is handled by the shared GPS recovery
        # controller. Normal mission execution must not advance while
        # its emergency landing branch is active.
        # ==========================================================

        if self.executor.gps_emergency_landing:

            self.last_tick_time = (
                time.monotonic()
            )

            return (
                py_trees.common.Status.RUNNING
            )

        # ==========================================================
        # PRIORITY 3:
        # GPS recovery
        # ==========================================================
        #
        # GPS loss affects navigation actions only.
        # ==========================================================

        if (
            self.task["type"] == "navigate"
            and self.executor.gps_recovery_active
        ):

            if not self.gps_pause_reported:

                self.gps_pause_reported = True

                self.executor.on_task_paused(
                    self.task,
                    reason="GPS_LOST",
                )

            # Recovery time is deliberately excluded from the
            # navigation action's logical execution duration.
            self.last_tick_time = (
                time.monotonic()
            )

            return (
                py_trees.common.Status.RUNNING
            )

        # ----------------------------------------------------------
        # GPS recovery completed
        # ----------------------------------------------------------

        if self.gps_pause_reported:

            self.gps_pause_reported = False

            self.executor.get_logger().info(
                f"BT NODE RESUMING: "
                f"{self.executor.node_name(self.task)}"
            )

        # ==========================================================
        # Normal mission-action execution
        # ==========================================================

        now = (
            time.monotonic()
        )

        if self.last_tick_time is None:

            self.last_tick_time = now

        elapsed = (
            now
            - self.last_tick_time
        )

        self.last_tick_time = now

        self.remaining_duration -= (
            elapsed
        )

        # ----------------------------------------------------------
        # Action still executing
        # ----------------------------------------------------------

        if self.remaining_duration > 0.0:

            return (
                py_trees.common.Status.RUNNING
            )

        # ----------------------------------------------------------
        # Action completed
        # ----------------------------------------------------------

        if not self.completed:

            self.completed = True

            self.remaining_duration = 0.0

            self.executor.on_task_completed(
                self.task
            )

        return (
            py_trees.common.Status.SUCCESS
        )

    # ==============================================================
    # Termination
    # ==============================================================

    def terminate(
        self,
        new_status,
    ):
        """
        Reset timing reference whenever py_trees stops or invalidates
        this behaviour.

        The remaining logical duration is deliberately preserved so
        an interrupted task does not restart from zero.
        """

        self.last_tick_time = None
