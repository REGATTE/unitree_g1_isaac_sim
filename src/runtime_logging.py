"""Shared logging helpers for the Unitree G1 Isaac Sim runtime."""

from __future__ import annotations

import logging


ROOT_LOGGER_NAME = "unitree_g1_isaac_sim"
_DEFAULT_LOG_FORMAT = "%(asctime)s [%(levelname)s] [%(name)s] %(message)s"


def configure_logging(level: int = logging.INFO) -> None:
    """Configure process-wide logging once for the simulator runtime."""
    root_logger = logging.getLogger()
    if root_logger.handlers:
        root_logger.setLevel(level)
        return
    logging.basicConfig(level=level, format=_DEFAULT_LOG_FORMAT)


def get_logger(name: str | None = None) -> logging.Logger:
    """Return a repository logger under the shared namespace root."""
    if not name:
        return logging.getLogger(ROOT_LOGGER_NAME)
    return logging.getLogger(f"{ROOT_LOGGER_NAME}.{name}")
