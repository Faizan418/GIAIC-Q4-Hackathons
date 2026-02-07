"""Query model for the RAG Chatbot API."""

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


class QueryRequest(BaseModel):
    """Request model for query endpoint."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The text content of the user's query",
    )
    context: Optional[str] = Field(
        None, max_length=2000, description="Additional context provided with the query"
    )
    user_id: Optional[str] = Field(
        None, description="Identifier for the user making the query"
    )
    conversation_id: Optional[str] = Field(
        None,
        description="ID of the conversation thread if continuing existing conversation",
    )


class Query(BaseModel):
    """Model representing a user's query."""

    query_text: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The text content of the user's query",
    )
    context: Optional[str] = Field(
        None, max_length=2000, description="Additional context provided with the query"
    )
    user_id: Optional[str] = Field(
        None, description="Identifier for the user making the query"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow, description="When the query was submitted"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "query_text": "What is Physical AI?",
                "context": "I want to understand the basic concept",
                "user_id": "user123",
                "timestamp": "2025-12-16T10:00:00Z",
            }
        }
