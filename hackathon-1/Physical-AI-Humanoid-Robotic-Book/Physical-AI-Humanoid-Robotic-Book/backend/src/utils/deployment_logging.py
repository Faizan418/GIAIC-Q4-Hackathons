"""
Deployment Logging Configuration
Configuration for logging in the deployment system
"""

import logging
import logging.config
import os
from datetime import datetime
from typing import Any, Dict


def setup_deployment_logging() -> None:
    """
    Set up logging configuration for deployment operations
    """
    # Create logs directory if it doesn't exist
    logs_dir = "logs"
    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir, exist_ok=True)

    # Define the logging configuration
    logging_config: Dict[str, Any] = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "standard": {"format": "%(asctime)s [%(levelname)s] %(name)s: %(message)s"},
            "detailed": {
                "format": "%(asctime)s [%(levelname)s] %(name)s %(funcName)s:%(lineno)d: %(message)s"
            },
            "json": {
                "format": '{"timestamp": "%(asctime)s", "level": "%(levelname)s", "logger": "%(name)s", "message": "%(message)s", "module": "%(module)s", "function": "%(funcName)s", "line": %(lineno)d}'
            },
        },
        "handlers": {
            "console": {
                "level": "INFO",
                "class": "logging.StreamHandler",
                "formatter": "standard",
                "stream": "ext://sys.stdout",
            },
            "file": {
                "level": "DEBUG",
                "class": "logging.handlers.RotatingFileHandler",
                "formatter": "detailed",
                "filename": f'logs/deployment_{datetime.now().strftime("%Y%m%d")}.log',
                "maxBytes": 10485760,  # 10MB
                "backupCount": 5,
            },
            "error_file": {
                "level": "ERROR",
                "class": "logging.handlers.RotatingFileHandler",
                "formatter": "detailed",
                "filename": f'logs/deployment_errors_{datetime.now().strftime("%Y%m%d")}.log',
                "maxBytes": 10485760,  # 10MB
                "backupCount": 10,
            },
            "deployment_file": {
                "level": "INFO",
                "class": "logging.handlers.RotatingFileHandler",
                "formatter": "detailed",
                "filename": f'logs/deployments_{datetime.now().strftime("%Y%m%d")}.log',
                "maxBytes": 10485760,  # 10MB
                "backupCount": 10,
            },
        },
        "loggers": {
            "deployment": {
                "handlers": ["console", "file", "deployment_file"],
                "level": "INFO",
                "propagate": False,
            },
            "deployment.service": {
                "handlers": ["console", "file", "deployment_file"],
                "level": "INFO",
                "propagate": False,
            },
            "deployment.api": {
                "handlers": ["console", "file", "deployment_file"],
                "level": "INFO",
                "propagate": False,
            },
            "deployment.monitor": {
                "handlers": ["console", "file", "deployment_file"],
                "level": "INFO",
                "propagate": False,
            },
        },
        "root": {"level": "INFO", "handlers": ["console", "file", "error_file"]},
    }

    # Apply the logging configuration
    logging.config.dictConfig(logging_config)


def get_deployment_logger(name: str) -> logging.Logger:
    """
    Get a configured logger for deployment operations
    """
    logger = logging.getLogger(name)
    return logger


# Initialize the logging configuration when this module is imported
setup_deployment_logging()


# Example usage
if __name__ == "__main__":
    # Test the logging configuration
    logger = get_deployment_logger("deployment.test")
    logger.info("Deployment logging configuration test")
    logger.error("This is a test error message")
