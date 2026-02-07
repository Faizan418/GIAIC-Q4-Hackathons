"""Conversation Service for the Book Intelligence Agent."""

import logging
from datetime import datetime
from typing import Any, Dict, List, Optional
from uuid import uuid4

from backend.db.neon_postgres import db
from backend.models.conversation import (
    ConversationMessage,
    ConversationThread,
    UserAnalytics,
)

logger = logging.getLogger(__name__)


class ConversationService:
    """Service class to handle conversation-related operations."""

    def __init__(self):
        """Initialize the conversation service."""

    async def create_or_update_conversation_thread(
        self,
        user_id: Optional[str] = None,
        title: str = "New Conversation",
        metadata: Optional[Dict[str, Any]] = None,
    ) -> ConversationThread:
        """Create a new conversation thread or update an existing one."""
        thread_id = str(uuid4())
        now = datetime.utcnow()

        # Use the database's upsert functionality to create or update the thread
        thread_id = await db.insert_conversation_thread(
            thread_id=thread_id,
            user_id=user_id,
            title=title,
            metadata=metadata or {},
            created_at=now,
            updated_at=now,
        )

        # Return the created/updated thread
        return ConversationThread(
            id=thread_id,
            user_id=user_id,
            created_at=now,
            updated_at=now,
            title=title,
            metadata=metadata or {},
        )

    async def get_conversation_thread(
        self, thread_id: str
    ) -> Optional[ConversationThread]:
        """Retrieve a conversation thread by ID."""
        thread_data = await db.get_conversation_thread(thread_id)
        if not thread_data:
            return None

        return ConversationThread(
            id=thread_data["id"],
            user_id=thread_data["user_id"],
            created_at=thread_data["created_at"],
            updated_at=thread_data["updated_at"],
            title=thread_data["title"],
            metadata=thread_data["metadata"],
        )

    async def create_or_update_conversation_message(
        self,
        conversation_id: str,
        role: str,
        content: str,
        context_used: Optional[Dict[str, Any]] = None,
        citations: Optional[Dict[str, Any]] = None,
    ) -> ConversationMessage:
        """Create a new conversation message or update an existing one."""
        message_id = str(uuid4())
        timestamp = datetime.utcnow()

        # Use the database's upsert functionality to create or update the message
        message_id = await db.insert_conversation_message(
            message_id=message_id,
            conversation_id=conversation_id,
            role=role,
            content=content,
            timestamp=timestamp,
            context_used=context_used or {},
            citations=citations or {},
        )

        # Return the created/updated message
        return ConversationMessage(
            id=message_id,
            conversation_id=conversation_id,
            role=role,
            content=content,
            timestamp=timestamp,
            context_used=context_used or {},
            citations=citations or {},
        )

    async def get_conversation_messages(
        self, conversation_id: str
    ) -> List[ConversationMessage]:
        """Retrieve all messages for a conversation."""
        messages_data = await db.get_conversation_messages(conversation_id)
        messages = []

        for msg_data in messages_data:
            message = ConversationMessage(
                id=msg_data["id"],
                conversation_id=msg_data["conversation_id"],
                role=msg_data["role"],
                content=msg_data["content"],
                timestamp=msg_data["timestamp"],
                context_used=msg_data["context_used"],
                citations=msg_data["citations"],
            )
            messages.append(message)

        return messages

    async def create_or_update_user_analytics(
        self,
        user_id: Optional[str],
        session_id: str,
        query: str,
        response_time: float,
        was_answered: bool,
        used_selected_snippet: bool = False,
        satisfaction_score: Optional[int] = None,
        was_accurate: Optional[bool] = None,
    ) -> UserAnalytics:
        """Create or update user analytics record."""
        analytics_id = str(uuid4())
        timestamp = datetime.utcnow()

        # Use the database's upsert functionality to create or update analytics
        analytics_id = await db.insert_user_analytics(
            analytics_id=analytics_id,
            user_id=user_id,
            session_id=session_id,
            query=query,
            response_time=response_time,
            satisfaction_score=satisfaction_score,
            timestamp=timestamp,
            was_answered=was_answered,
            was_accurate=was_accurate,
            used_selected_snippet=used_selected_snippet,
        )

        # Return the created/updated analytics record
        return UserAnalytics(
            id=analytics_id,
            user_id=user_id,
            session_id=session_id,
            query=query,
            response_time=response_time,
            satisfaction_score=satisfaction_score,
            timestamp=timestamp,
            was_answered=was_answered,
            was_accurate=was_accurate,
            used_selected_snippet=used_selected_snippet,
        )

    async def get_conversation_history(
        self, conversation_id: str, limit: int = 50
    ) -> List[ConversationMessage]:
        """Retrieve conversation history for context."""
        return await self.get_conversation_messages(conversation_id)

    async def get_user_analytics(self, user_id: str) -> List[UserAnalytics]:
        """Retrieve user analytics by user ID."""
        analytics_data = await db.get_user_analytics_by_user_id(user_id)
        analytics_list = []

        for data in analytics_data:
            analytics = UserAnalytics(
                id=data["id"],
                user_id=data["user_id"],
                session_id=data["session_id"],
                query=data["query"],
                response_time=data["response_time"],
                satisfaction_score=data["satisfaction_score"],
                timestamp=data["timestamp"],
                was_answered=data["was_answered"],
                was_accurate=data["was_accurate"],
                used_selected_snippet=data["used_selected_snippet"],
            )
            analytics_list.append(analytics)

        return analytics_list
