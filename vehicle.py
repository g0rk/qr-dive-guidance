# vehicle.py — MAVSDK wrapper. All drone commands are issued through this module.
# States must NEVER import mavsdk directly; they interact only via Vehicle methods.

from __future__ import annotations

from typing import List
import asyncio
import logging

from mavsdk.offboard import Attitude
from mavsdk import System

from telemetry import TelemetryStore

import config

logger = logging.getLogger("VEHICLE")

class VehicleCommandError(Exception):
    """
    Raised when a MAVSDK command fails.
    """

class Vehicle:
    """High level drone interface wrapping MAVSDK."""

    def __init__(self, telemetry_store: TelemetryStore) -> None:
        self._telemetry_store = telemetry_store
        self._system = System(port=50051, sysid=1)
        self._offboard_active = False

    # Connection
    async def connect(self) -> None:
        """Connect to the autopilot and wait for link establishment."""
        logger.info("Connecting to %s …", config.SYSTEM_ADDRESS)
        await self._system.connect(system_address=config.SYSTEM_ADDRESS)
        
        async for state in self._system.core.connection_state():
            if state.is_connected:
                logger.info("Connected to autopilot")
                break

    async def wait_until_ready(self) -> None:
        """Block until GPS and home position health checks pass."""
        logger.info("Waiting for GPS fix and home position …")
        async for health in self._system.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                logger.info("Vehicle ready (GPS OK, home position set)")
                break

    # Telemetry tasks
    def telemetry_tasks(self) -> List[asyncio.Task]:
        """
        Spawn background tasks that feed the TelemetryStore.
        Returns the list of tasks so the caller can cancel them on shutdown.
        """
        return [
            asyncio.create_task(self._stream_position(), name="tel_position"),
            asyncio.create_task(self._stream_velocity(), name="tel_velocity"),
            asyncio.create_task(self._stream_heading(), name="tel_heading"),
            asyncio.create_task(self._stream_armed(), name="tel_armed"),
            asyncio.create_task(self._stream_flight_mode(), name="tel_flight_mode"),
            asyncio.create_task(self._stream_attitude(), name="tel_attitude"),
        ]

    async def _stream_position(self) -> None:
        logger.debug("Starting position stream")
        try:
            async for pos in self._system.telemetry.position():
                self._telemetry_store.update_position(
                    latitude_deg=pos.latitude_deg,
                    longitude_deg=pos.longitude_deg,
                    abs_alt_m=pos.absolute_altitude_m,
                    rel_alt_m=pos.relative_altitude_m,
                )
        except Exception as e:
            logger.error("Position stream failed: %s — restarting", e)
            await asyncio.sleep(1.0)
            asyncio.create_task(self._stream_position(), name="tel_position_restart")

    async def _stream_velocity(self) -> None:
        logger.debug("Starting velocity stream")
        try:
            async for vel in self._system.telemetry.velocity_ned():
                self._telemetry_store.update_velocity(
                    vel_north=vel.north_m_s,
                    vel_east=vel.east_m_s,
                    vel_down=vel.down_m_s,
                )
        except Exception as e:
            logger.error("Velocity stream failed: %s — restarting", e)
            await asyncio.sleep(1.0)
            asyncio.create_task(self._stream_velocity(), name="tel_velocity_restart")

    async def _stream_heading(self) -> None:
        logger.debug("Starting heading stream")
        try:
            async for heading in self._system.telemetry.heading():
                self._telemetry_store.update_heading(heading.heading_deg)
        except Exception as e:
            logger.error("Heading stream failed: %s — restarting", e)
            await asyncio.sleep(1.0)
            asyncio.create_task(self._stream_heading(), name="tel_heading_restart")

    async def _stream_armed(self) -> None:
        logger.debug("Starting armed stream")
        try:
            async for armed in self._system.telemetry.armed():
                self._telemetry_store.update_armed(armed)
        except Exception as e:
            logger.error("Armed stream failed: %s — restarting", e)
            await asyncio.sleep(1.0)
            asyncio.create_task(self._stream_armed(), name="tel_armed_restart")

    async def _stream_flight_mode(self) -> None:
        logger.debug("Starting flight mode stream")
        try:
            async for mode in self._system.telemetry.flight_mode():
                self._telemetry_store.update_flight_mode(str(mode))
        except Exception as e:
            logger.error("Flight mode stream failed: %s — restarting", e)
            await asyncio.sleep(1.0)
            asyncio.create_task(
                self._stream_flight_mode(), name="tel_flight_mode_restart"
            )

    async def _stream_attitude(self) -> None:
        logger.debug("Starting attitude stream")
    
        try:
            async for euler in self._system.telemetry.attitude_euler():
                self._telemetry_store.update_attitude(
                    roll_deg=euler.roll_deg,
                    pitch_deg=euler.pitch_deg,
                    heading_deg=euler.yaw_deg,
                )
    
        except Exception as e:
            logger.error("Attitude stream failed: %s — restarting", e)
            await asyncio.sleep(1.0)
            asyncio.create_task(
                self._stream_attitude(),
                name="tel_attitude_restart",
            )

    # Commands
    async def goto_location(self, lat: float, lon: float, abs_alt_m: float, yaw_deg: float = 0.0) -> None:
        """Command the autopilot to fly to a geographic location."""
        logger.info(
            "goto_location(%.6f, %.6f, alt=%.1fm, yaw=%.1f°)",
            lat, lon, abs_alt_m, yaw_deg,
        )
        try:
            await self._system.action.goto_location(lat, lon, abs_alt_m, yaw_deg)
            logger.info("goto_location command accepted")
        except Exception as e:
            logger.error("goto_location failed: %s", e)
            raise VehicleCommandError(f"goto_location failed: {e}") from e

    def field_elevation_m(self) -> float:
        """
        Elevation of the launch site above sea level, in metres.

        MAVSDK's `position` reports both the absolute (AMSL) and the relative
        altitude; the difference between them is the AMSL elevation of the
        home point.
        """
        tel = self._telemetry_store.get()
        return tel.abs_alt_m - tel.rel_alt_m

    async def goto_location_rel(self, lat: float, lon: float, rel_alt_m: float,
                                yaw_deg: float = 0.0) -> None:
        """
        goto_location, but taking a RELATIVE altitude.

        ⚠️ WHY THIS EXISTS: MAVSDK's `action.goto_location()` expects an
           **ABSOLUTE (AMSL)** altitude. Every other altitude in this project
           is RELATIVE: TAKEOFF_ALTITUDE_M (MAVSDK's takeoff already takes a
           relative one), DIVE_MIN_ENTRY_ALTITUDE_M, DIVE_PULL_UP_ALTITUDE_M
           and all the safety checks work off `tel.rel_alt_m`. The rulebook
           speaks in relative terms too (p.18: "at least 100 m relative to the
           runway").

           Confusing the two fails SILENTLY and fatally: the approach commands
           100 m AMSL while the dive permission tests rel_alt >= threshold. At
           any site NOT at sea level, rel_alt stays permanently lower and the
           dive NEVER triggers.

           ⚠️ IT BITES IMMEDIATELY IN SIMULATION: the PX4 SITL default world
           is Zurich, at **488 m AMSL**. Flying to "100 m AMSL" without the
           conversion means commanding a point 388 m BELOW the ground.

           The rule: convert at the MAVSDK boundary, keep config relative.
        """
        tel = self._telemetry_store.get()
        if tel.last_update_time == 0.0:
            raise VehicleCommandError(
                "goto_location_rel: no telemetry yet, so the field elevation "
                "is unknown - a relative altitude cannot be converted to absolute"
            )

        elevation = self.field_elevation_m()
        abs_alt_m = elevation + rel_alt_m
        logger.info(
            "goto_location_rel: rel=%.1fm + field=%.1fm AMSL -> absolute=%.1fm",
            rel_alt_m, elevation, abs_alt_m,
        )
        await self.goto_location(lat, lon, abs_alt_m, yaw_deg)

    async def set_attitude(self, roll_deg: float, pitch_deg: float, yaw_rate_deg_s: float, thrust: float) -> None:
        try:
            attitude = Attitude(roll_deg, pitch_deg, yaw_rate_deg_s, thrust)
            await self._system.offboard.set_attitude(attitude)

            if not self._offboard_active:
                await self._ensure_offboard_started()

        except Exception as e:
            logger.error("set_attitude failed: %s", e)
            raise VehicleCommandError(f"set_attitude failed: {e}") from e

    async def _ensure_offboard_started(self) -> None:
        """Start offboard mode, tolerating 'already active' errors only."""
        try:
            await self._system.offboard.start()
            self._offboard_active = True
            logger.info("Offboard mode started successfully.")
        except Exception as e:
            err_str = str(e).lower()
            if "already" in err_str or "command 176" in err_str:
                # Drone already in offboard — this is fine
                self._offboard_active = True
                logger.debug("Offboard already active, suppressing: %s", e)
            else:
                # Real failure — do NOT set flag, will retry next tick
                logger.warning("Offboard start failed (will retry): %s", e)
                raise VehicleCommandError(f"Offboard start failed: {e}") from e

    async def arm(self) -> None:
        """Arm the vehicle."""
        logger.info("Sending arm command")
        try:
            await self._system.action.arm()
            logger.info("Arm command accepted")
        except Exception as e:
            logger.error("Arm command failed: %s", e)
            raise VehicleCommandError(f"Arm failed: {e}") from e

    async def takeoff(self, alt_m: float) -> None:
        """Command takeoff to the given relative altitude."""
        logger.info("Commanding takeoff to %.1f m", alt_m)
        try:
            await self._system.action.set_takeoff_altitude(alt_m)
            await self._system.action.takeoff()
            logger.info("Takeoff command accepted")
        except Exception as e:
            logger.error("Takeoff command failed: %s", e)
            raise VehicleCommandError(f"Takeoff failed: {e}") from e

    async def rtl(self) -> None:
        """Command return-to-launch."""
        logger.info("Commanding RTL")
        try:
            await self._system.action.return_to_launch()
            logger.info("RTL command accepted")
        except Exception as e:
            logger.error("RTL command failed: %s", e)
            raise VehicleCommandError(f"RTL failed: {e}") from e

    async def hold(self) -> None:
        """Command HOLD/LOITER mode."""
        logger.info("Commanding HOLD")
        try:
            await self._system.action.hold()
            logger.info("HOLD command accepted")
        except Exception as e:
            logger.error("HOLD command failed: %s", e)
            raise VehicleCommandError(f"HOLD failed: {e}") from e
