from __future__ import annotations

import multiprocessing as mp
import logging
import asyncio
import json

from websockets.server import WebSocketServerProtocol
import websockets

from logger import setup_logger

import config

logger = logging.getLogger("COMM")

class CommunicationProcess(mp.Process):
    """
    Standalone process that hosts a WebSocket server.

    Send a JSON message to the GUI by putting a dict on status_queue.
    Receive commands from the GUI via command_queue.
    """

    def __init__(self, command_queue: mp.Queue, status_queue: mp.Queue, host: str = config.WS_HOST, port: int = config.WS_PORT) -> None:
        super().__init__(name="comm", daemon=True)
        self.command_queue = command_queue
        self.status_queue = status_queue
        self.host = host
        self.port = port

    # Process entry point
    def run(self) -> None:
        setup_logger(level=logging.INFO, log_to_file=False)
        try:
            asyncio.run(self._serve())
        except KeyboardInterrupt:
            pass        

    # Async core
    async def _serve(self) -> None:
        self._clients: set[WebSocketServerProtocol] = set()

        async with websockets.serve(self._handle_client, self.host, self.port):
            logger.info("WebSocket server listening on ws://%s:%d", self.host, self.port)
            await asyncio.gather(
                self._broadcast_loop(),
            )

    async def _handle_client(self, ws: WebSocketServerProtocol) -> None:
        """Handle a single GUI client connection."""
        self._clients.add(ws)
        logger.info("GUI connected (%d client(s))", len(self._clients))
        try:
            async for raw in ws:
                try:
                    # msg = json.loads(raw)
                    # logger.info("Received from GUI: %s", raw)
                    self.command_queue.put_nowait(raw)
                except json.JSONDecodeError:
                    logger.warning("Non-JSON message from GUI: %r", raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            self._clients.discard(ws)
            logger.info("GUI disconnected (%d client(s) remaining)", len(self._clients))

    async def _broadcast_loop(self) -> None:
        """Poll status_queue and broadcast new messages to all connected clients."""
        loop = asyncio.get_running_loop()
        while True:
            # Non-blocking poll — yields control every 50 ms when queue is empty
            try:
                msg = await loop.run_in_executor(None, self._poll_status)
            except Exception as e:
                logger.warning("status_queue read error: %s", e)
                await asyncio.sleep(0.05)
                continue

            if msg is None:
                await asyncio.sleep(0.05)
                continue

            if self._clients:
                payload = json.dumps(msg)
                await asyncio.gather(
                    *[self._send_safe(c, payload) for c in list(self._clients)],
                    return_exceptions=True,
                )

    def _poll_status(self) -> dict | None:
        """Blocking call (runs in thread executor) — returns None if queue empty."""
        try:
            return self.status_queue.get(timeout=0.05)
        except Exception:
            return None

    @staticmethod
    async def _send_safe(ws: WebSocketServerProtocol, payload: str) -> None:
        try:
            await ws.send(payload)
        except websockets.ConnectionClosed:
            pass
