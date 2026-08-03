from rich.logging import RichHandler
import logging
import os

LOG_FORMAT = "%(asctime)s | %(levelname)-8s | %(name)-13s | %(message)s"
DATE_FORMAT = "%H:%M:%S"

class SkipTickFilter(logging.Filter):
    def filter(self, record):
        return not getattr(record, "tick", False)

def setup_logger(level: int = logging.INFO, log_to_file: bool = False, log_file: str = "logs/mission.log") -> None:
    logging.getLogger("websockets.server").setLevel(logging.WARNING)
    logging.getLogger("websockets.client").setLevel(logging.WARNING)
    
    root = logging.getLogger()
    root.setLevel(level)
    root.handlers.clear()

    console = RichHandler(rich_tracebacks=True, markup=True, show_path=False, log_time_format="[%H:%M:%S]")
    console.setLevel(level)
    console.addFilter(SkipTickFilter())

    console_formatter = logging.Formatter("%(name)-13s | %(message)s")
    console.setFormatter(console_formatter)
    
    root.addHandler(console)

    if log_to_file:
        log_dir = os.path.dirname(log_file)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        file_handler = logging.FileHandler(log_file, mode="a", encoding="utf-8")
        file_handler.setLevel(level)
        file_handler.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=DATE_FORMAT))
        root.addHandler(file_handler)