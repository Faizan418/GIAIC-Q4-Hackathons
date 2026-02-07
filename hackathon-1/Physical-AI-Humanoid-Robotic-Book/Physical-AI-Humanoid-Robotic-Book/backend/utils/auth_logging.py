"""Authentication logging utilities for the RAG Chatbot API."""

import logging
from datetime import datetime
from enum import Enum
from typing import Optional


class AuthLogEvent(str, Enum):
    """Enumeration of authentication-related log events."""

    USER_REGISTRATION_START = "user_registration_start"
    USER_REGISTRATION_SUCCESS = "user_registration_success"
    USER_REGISTRATION_FAILURE = "user_registration_failure"
    USER_LOGIN_START = "user_login_start"
    USER_LOGIN_SUCCESS = "user_login_success"
    USER_LOGIN_FAILURE = "user_login_failure"
    USER_LOGOUT = "user_logout"
    PROFILE_CREATE_START = "profile_create_start"
    PROFILE_CREATE_SUCCESS = "profile_create_success"
    PROFILE_CREATE_FAILURE = "profile_create_failure"
    PROFILE_UPDATE_START = "profile_update_start"
    PROFILE_UPDATE_SUCCESS = "profile_update_success"
    PROFILE_UPDATE_FAILURE = "profile_update_failure"
    TOKEN_CREATION = "token_creation"
    TOKEN_VALIDATION_START = "token_validation_start"
    TOKEN_VALIDATION_SUCCESS = "token_validation_success"
    TOKEN_VALIDATION_FAILURE = "token_validation_failure"
    AUTHORIZATION_CHECK = "authorization_check"


# Create a dedicated logger for auth operations
auth_logger = logging.getLogger("auth")
auth_logger.setLevel(logging.INFO)

# Create a handler for auth logs (separate from general app logs)
if not auth_logger.handlers:
    # Create a file handler for auth logs
    from logging.handlers import RotatingFileHandler

    handler = RotatingFileHandler(
        "auth.log", maxBytes=10 * 1024 * 1024, backupCount=5
    )  # 10MB
    handler.setLevel(logging.INFO)

    # Create a formatter for auth logs
    formatter = logging.Formatter("%(asctime)s - AUTH - %(levelname)s - %(message)s")
    handler.setFormatter(formatter)

    # Add the handler to the auth logger
    auth_logger.addHandler(handler)


def log_auth_event(
    event: AuthLogEvent,
    user_id: Optional[str] = None,
    email: Optional[str] = None,
    details: Optional[dict] = None,
    ip_address: Optional[str] = None,
):
    """
    Log an authentication-related event.

    Args:
        event: The authentication event to log
        user_id: The ID of the user involved (if applicable)
        email: The email of the user involved (if applicable)
        details: Additional details about the event
        ip_address: The IP address of the request (if applicable)
    """
    log_data = {
        "event": event.value,
        "timestamp": datetime.utcnow().isoformat(),
        "user_id": user_id,
        "email": email,
        "ip_address": ip_address,
        "details": details or {},
    }

    # Log the event
    auth_logger.info(f"AUTH_EVENT: {log_data}")


def log_auth_error(
    event: AuthLogEvent,
    error: Exception,
    user_id: Optional[str] = None,
    email: Optional[str] = None,
    details: Optional[dict] = None,
):
    """
    Log an authentication-related error.

    Args:
        event: The authentication event that resulted in an error
        error: The error that occurred
        user_id: The ID of the user involved (if applicable)
        email: The email of the user involved (if applicable)
        details: Additional details about the event
    """
    log_data = {
        "event": event.value,
        "timestamp": datetime.utcnow().isoformat(),
        "user_id": user_id,
        "email": email,
        "error_type": type(error).__name__,
        "error_message": str(error),
        "details": details or {},
    }

    # Log the error
    auth_logger.error(f"AUTH_ERROR: {log_data}")
