"""Conversation API endpoints for the Book Intelligence Agent."""

from typing import List, Optional

from fastapi import APIRouter, HTTPException

from backend.models.conversation import ConversationMessage, ConversationThread
from backend.services.conversation_service import ConversationService

router = APIRouter(prefix="/api/conversation", tags=["conversation"])


@router.post("/thread", response_model=ConversationThread)
async def create_conversation_thread(
    user_id: Optional[str] = None,
    title: str = "New Conversation",
    metadata: Optional[dict] = None,
):
    """Create a new conversation thread or update an existing one."""
    try:
        service = ConversationService()
        thread = await service.create_or_update_conversation_thread(
            user_id=user_id, title=title, metadata=metadata
        )
        return thread
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to create conversation thread: {str(e)}"
        )


@router.get("/thread/{thread_id}", response_model=ConversationThread)
async def get_conversation_thread(thread_id: str):
    """Retrieve a conversation thread by ID."""
    try:
        service = ConversationService()
        thread = await service.get_conversation_thread(thread_id)
        if not thread:
            raise HTTPException(status_code=404, detail="Conversation thread not found")
        return thread
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to retrieve conversation thread: {str(e)}"
        )


@router.post("/message", response_model=ConversationMessage)
async def create_conversation_message(
    conversation_id: str,
    role: str,
    content: str,
    context_used: Optional[dict] = None,
    citations: Optional[dict] = None,
):
    """Create a new conversation message or update an existing one."""
    # Validate role
    if role not in ["user", "assistant"]:
        raise HTTPException(
            status_code=400, detail="Role must be either 'user' or 'assistant'"
        )

    try:
        service = ConversationService()
        message = await service.create_or_update_conversation_message(
            conversation_id=conversation_id,
            role=role,
            content=content,
            context_used=context_used,
            citations=citations,
        )
        return message
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to create conversation message: {str(e)}"
        )


@router.get("/thread/{thread_id}/messages", response_model=List[ConversationMessage])
async def get_conversation_messages(thread_id: str):
    """Retrieve all messages for a conversation."""
    try:
        service = ConversationService()
        messages = await service.get_conversation_messages(thread_id)
        return messages
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to retrieve conversation messages: {str(e)}",
        )


@router.get("/{conversation_id}/history", response_model=List[ConversationMessage])
async def get_conversation_history(conversation_id: str, limit: int = 50):
    """Retrieve conversation history for context."""
    try:
        service = ConversationService()
        history = await service.get_conversation_history(conversation_id, limit)
        return history
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to retrieve conversation history: {str(e)}"
        )
