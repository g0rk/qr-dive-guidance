from __future__ import annotations

import asyncio
import logging

from mavsdk import System

logger = logging.getLogger("TARGET")

MAVSDK_PORT = "udp://:14542"


async def target_stream(out_queue: asyncio.Queue, hz: float = 20.0) -> None:
    """
    Put one dict on out_queue per tick:

        {
            "type": "target",
            "lat":  float,   # degrees
            "lon":  float,   # degrees
            "alt":  float,   # metres (AMSL)
            "vx":   float,   # m/s (north)
            "vy":   float,   # m/s (east)
            "vz":   float,   # m/s (down)
            "roll":  float,  # degrees
            "pitch": float,  # degrees
            "yaw":   float,  # degrees
        }

    On a dropped connection or missing data it logs and retries.
    Cancellation from outside arrives as asyncio.CancelledError.
    """
    dt = 1.0 / hz

    while True:
        drone = System(port=50052, sysid=2)
        try:
            logger.info("Connecting to target drone on %s ...", MAVSDK_PORT)
            await drone.connect(system_address=MAVSDK_PORT)

            # Wait for the connection to be confirmed.
            async for state in drone.core.connection_state():
                if state.is_connected:
                    logger.info("Target drone connected.")
                    break

            await _stream_loop(drone, out_queue, dt)

        except asyncio.CancelledError:
            logger.info("target_stream cancelled.")
            raise

        except Exception as e:
            logger.warning("target_stream error: %s — retrying in 3s", e)
            await asyncio.sleep(3.0)


async def _stream_loop(
    drone: System,
    out_queue: asyncio.Queue,
    dt: float,
) -> None:
    """Poll telemetry from the connected drone at a fixed rate."""

    # Each stream is its own async generator; a tiny holder keeps the
    # latest value from each.
    pos   = _Holder()
    vel   = _Holder()
    euler = _Holder()

    # Start the stream tasks in the background.
    tasks = [
        asyncio.create_task(_pos_stream(drone, pos),   name="t_pos"),
        asyncio.create_task(_vel_stream(drone, vel),   name="t_vel"),
        asyncio.create_task(_att_stream(drone, euler), name="t_att"),
    ]

    try:
        while True:
            await asyncio.sleep(dt)

            # Skip this tick until every stream has produced something.
            if not (pos.value and vel.value and euler.value):
                continue

            payload = {
                "type": "target",
                "lat":   pos.value.latitude_deg,
                "lon":   pos.value.longitude_deg,
                "alt":   pos.value.absolute_altitude_m,
                "vx":    vel.value.north_m_s,
                "vy":    vel.value.east_m_s,
                "vz":    vel.value.down_m_s,
                "roll":  euler.value.roll_deg,
                "pitch": euler.value.pitch_deg,
                "yaw":   euler.value.yaw_deg,
            }

            # If the queue is full, drop the oldest entry and push the new
            # one - stale target positions are worthless.
            if out_queue.full():
                try:
                    out_queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass

            await out_queue.put(payload)

    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


# -- Telemetry stream helpers ------------------------------------------------

class _Holder:
    """Minimal container holding the most recent telemetry value."""
    __slots__ = ("value",)
    def __init__(self) -> None:
        self.value = None


async def _pos_stream(drone: System, holder: _Holder) -> None:
    async for pos in drone.telemetry.position():
        holder.value = pos


async def _vel_stream(drone: System, holder: _Holder) -> None:
    async for vel in drone.telemetry.velocity_ned():
        holder.value = vel


async def _att_stream(drone: System, holder: _Holder) -> None:
    async for att in drone.telemetry.attitude_euler():
        holder.value = att