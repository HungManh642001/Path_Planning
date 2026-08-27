"""Logger configuration."""

import logging
import os
import sys
from logging.handlers import RotatingFileHandler


def setup_logging(
    service_name: str,
    log_file: str = "app.log",
) -> logging.Logger:
    """Set up logging configuration for the application.

    Args:
        service_name: Name of the service or application.
        log_file: Path to the log file.

    Returns:
        The configured logger instance.
    """
    # Create a logger
    logger = logging.getLogger(service_name)
    logger.setLevel(logging.DEBUG)  # Set the default logging level

    # Create a console handler for output to stdout
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)  # Set console logging level

    # Create a rotating file handler for output to a log file
    if log_dir := os.path.dirname(log_file):
        os.makedirs(log_dir, exist_ok=True)
    file_handler = RotatingFileHandler(
        log_file, maxBytes=5 * 1024 * 1024, backupCount=5, encoding="utf-8"
    )  # 5 MB per file, keep 5 backups
    file_handler.setLevel(logging.DEBUG)  # Set file logging level

    # Create a formatter and set it for both handlers
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    console_handler.setFormatter(formatter)
    file_handler.setFormatter(formatter)

    # Add handlers to the logger
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)
    return logger
