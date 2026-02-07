"""Conversation models for the Book Intelligence Agent."""

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, Optional


@dataclass
class ConversationThread:
    """Model representing a conversation thread."""

    id: str
    user_id: Optional[str]
    created_at: datetime
    updated_at: datetime
    title: str
    metadata: Optional[Dict[str, Any]]

    def __post_init__(self):
        if isinstance(self.created_at, str):
            self.created_at = datetime.fromisoformat(
                self.created_at.replace("Z", "+00:00")
            )
        if isinstance(self.updated_at, str):
            self.updated_at = datetime.fromisoformat(
                self.updated_at.replace("Z", "+00:00")
            )


@dataclass
class ConversationMessage:
    """Model representing a message within a conversation."""

    id: str
    conversation_id: str
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime
    context_used: Optional[Dict[str, Any]]
    citations: Optional[Dict[str, Any]]

    def __post_init__(self):
        if isinstance(self.timestamp, str):
            self.timestamp = datetime.fromisoformat(
                self.timestamp.replace("Z", "+00:00")
            )


@dataclass
class UserAnalytics:
    """Model representing user analytics data."""

    id: str
    user_id: Optional[str]
    session_id: str
    query: str
    response_time: float
    satisfaction_score: Optional[int]
    timestamp: datetime
    was_answered: bool
    was_accurate: Optional[bool]
    used_selected_snippet: bool

    def __post_init__(self):
        if isinstance(self.timestamp, str):
            self.timestamp = datetime.fromisoformat(
                self.timestamp.replace("Z", "+00:00")
            )
