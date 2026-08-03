from __future__ import annotations

import multiprocessing as mp
import logging
import sys

from processes.ground_communication import CommunicationProcess
from processes.mission_controller import MissionController
from processes.perception import PerceptionProcess

from telemetry import TelemetryStore
from vehicle import Vehicle

from logger import setup_logger

import config

logger = logging.getLogger("MAIN")

def main() -> None:
    setup_logger(level=logging.INFO, log_to_file=False)
    logger.info("Starting Kamikaze Mission System")

    # Shared queues between processes
    comm_command_queue: mp.Queue = mp.Queue()   # GUI commands → mission
    comm_status_queue: mp.Queue = mp.Queue()    # Mission status → GUI

    perception_command_queue: mp.Queue = mp.Queue() # Mission → QR scanner
    perception_result_queue: mp.Queue = mp.Queue()  # QR scanner → mission

    telemetry_store = TelemetryStore()
    vehicle = Vehicle(telemetry_store)

    # Process 1: Communication (WebSocket)
    comm = CommunicationProcess(comm_command_queue, comm_status_queue)
    comm.start()
    logger.info("CommunicationProcess started (pid=%d)", comm.pid)

    # Process 2: QR Scanner
    perception = PerceptionProcess(perception_command_queue, perception_result_queue, yolo_model_path="best.pt")
    perception.start()
    logger.info("QRScannerProcess started (pid=%d)", perception.pid)

    # Process 3: Mission
    mission = MissionController(vehicle, telemetry_store, comm_command_queue, comm_status_queue, perception_command_queue, perception_result_queue)
    mission.start()
    logger.info("MissionController started (pid=%d)", mission.pid)

    try:
        mission.join()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt — stopping all processes")
    finally:
        perception.put_nowait({"cmd": "kill"})
        for proc in (mission, perception, comm):
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=3)
        logger.info("All processes stopped")
        sys.exit(0)

if __name__ == "__main__":
    main()