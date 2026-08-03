# mission_controller.py — FSM loop, state transitions, and safety polling.

from __future__ import annotations

import multiprocessing as mp
import asyncio
import logging
import json

from logger import setup_logger

from safety import SafetyChecker

from states.abort_state import AbortState
from states.idle_state import IdleState
from states.base_state import BaseState

from utils.command_router import CommandRouter
from telemetry import TelemetryStore
from vehicle import Vehicle

import config

logger = logging.getLogger("MISSION")

class MissionController(mp.Process):
    """
    Standalone process running a finite state machine that controls the mission flow.

    Runs at config.LOOP_HZ. On each tick:
      1. Collect telemetry
      2. Execute safety checks (abort on failure)
      3. Call the current state's .update()
    """

    def __init__(self, vehicle: Vehicle, telemetry: TelemetryStore, comm_command_queue: mp.Queue, comm_status_queue: mp.Queue, perception_command_queue: mp.Queue, perception_result_queue: mp.Queue) -> None:
        super().__init__(name="mission", daemon=True)
        self.perception_command_queue = perception_command_queue
        self.perception_result_queue = perception_result_queue
        self.comm_command_queue = comm_command_queue
        self.comm_status_queue = comm_status_queue
        self.telemetry = telemetry
        self.vehicle = vehicle

        self.target: dict | None = None  # for simulation video

    # Process entry point
    def run(self) -> None:
        setup_logger(level=logging.INFO, log_to_file=False)
        try:
            asyncio.run(self._loop())
        except KeyboardInterrupt:
            pass

    # Async core
    async def _loop(self) -> None:
        self._router: CommandRouter  = CommandRouter()
        self._safety: SafetyChecker = SafetyChecker()
        self._state: BaseState = IdleState()
        self._running: bool = True

        tel_tasks: list[asyncio.Task] = []
        fsm_task: asyncio.Task | None = None

        try:
            # 1. Connect to autopilot
            await self.vehicle.connect()
            await self.vehicle.wait_until_ready()

            # 2. Start telemetry streams
            tel_tasks = self.vehicle.telemetry_tasks()
            logger.info("Telemetry streams started (%d tasks)", len(tel_tasks))

            # Brief pause to let initial telemetry populate
            await asyncio.sleep(2.0)

            # 3. Run FSM
            if config.AUTO_START:
                logger.info("AUTO_START enabled — launching mission controller")
                fsm_task = asyncio.create_task(self._run_fsm(), name="fsm")
                await fsm_task
            else:
                logger.info("AUTO_START disabled — waiting for manual trigger...")
                while True:
                    await asyncio.sleep(1.0)

        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received — shutting down...")

        except Exception as e:
            logger.critical("Unhandled exception in mission: %s", e, exc_info=True)

        finally:
            logger.info("Cleaning up async tasks...")

            # Cancel FSM task
            if fsm_task is not None and not fsm_task.done():
                fsm_task.cancel()
                try:
                    await fsm_task
                except asyncio.CancelledError:
                    pass

            # Cancel all telemetry tasks
            for task in tel_tasks:
                if not task.done():
                    task.cancel()

            # Gather cancellation results
            if tel_tasks:
                results = await asyncio.gather(*tel_tasks, return_exceptions=True)
                for task, result in zip(tel_tasks, results):
                    if isinstance(result, Exception) and not isinstance(result, asyncio.CancelledError):
                        logger.warning(
                            "Telemetry task %s raised on shutdown: %s",
                            task.get_name(), result,
                        )

            logger.info("Shutdown complete")

    async def _run_fsm(self) -> None:
        """Inner FSM tick loop — separated so it can be cleanly cancelled."""
        logger.info("Mission controller starting (%.0f Hz)", config.LOOP_HZ)
        await self._state.on_enter(self)

        dt = 1.0 / config.LOOP_HZ

        while self._running:
            await self._drain_commands() # Yeri değişebilir
            
            tel = self.telemetry.get()
            # safety_result = self._safety.check(tel)

            #if not safety_result.ok:
            if False:
                if self._state.name not in ("IDLE", "ABORT"):
                    logger.warning(
                        "Safety check failed in %s: %s",
                        self._state.name, safety_result.reason,
                    )
                    await self._change_state(AbortState())
            else:
                await self._state.update(self)

            await asyncio.sleep(dt)

        logger.info("Mission controller stopped")

    async def _drain_commands(self) -> None:
        """
        Comm kuyruğundaki tüm bekleyen JSON komutlarını tek tick'te işle.
 
        Kuyruğu bloklamadan okur (non-blocking); boş olduğunda durur.
        Kabul edilen komutlar _change_state() üzerinden FSM geçişini tetikler.
        Reddedilen komutlar loglanır, durumu etkilemez.
 
        Safety abort'a izin vermek için AbortState komutları her zaman işlenir,
        başka bir state'e geçiş yapılmış olsa bile kuyruk tamamen boşaltılır.
        """
        while True:
            try:
                raw: str = self.comm_command_queue.get_nowait()
            except Exception:
                # Queue is empty - multiprocessing.Queue.Empty
                break
 
            try:
                parsed = json.loads(raw)
                msg_type = parsed.get("type")
            except json.JSONDecodeError:
                logger.warning("Invalid JSON in command queue: %s", raw)
                continue

            if msg_type == "target":
                self.target = parsed
                # logger.info("Target updated: %s", self.target)
                continue

            result = self._router.resolve(raw, self._state.name)
            
            if result.ok:
                await self._change_state(result.new_state)
            else:
                logger.warning("Command rejected: %s", result.reason)

    # TODO
    def _push_status(self, payload: dict) -> None:
        """comm_status_queue'ya non-blocking olarak durum mesajı gönder."""
        try:
            self.comm_status_queue.put_nowait(payload)
        except Exception:
            pass  # Kuyruk doluysa sessizce geç

    async def _change_state(self, new_state: BaseState) -> None:
        """
        Transition from the current state to new_state.

        Handles on_exit() / on_enter() hooks safely. If on_enter()
        raises an exception, falls back to AbortState.
        """
        logger.info("State transition: %s -> %s", self._state.name, new_state.name)

        # Exit current state
        try:
            await self._state.on_exit(self)
        except Exception as e:
            logger.warning("%s.on_exit raised: %s", self._state.name, e)

        self._state = new_state

        # Enter new state — fall back to AbortState on failure
        try:
            await self._state.on_enter(self)
        except Exception as e:
            logger.error(
                "%s.on_enter raised: %s — falling back to AbortState",
                new_state.name, e,
            )
            self._state = AbortState()
            await self._state.on_enter(self)