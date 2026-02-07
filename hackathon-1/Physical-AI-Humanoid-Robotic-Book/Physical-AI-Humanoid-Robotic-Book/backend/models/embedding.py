"""Embedding model for the RAG Chatbot API."""

from datetime import datetime
from typing import Any, Dict, Optional

from pydantic import BaseModel, Field


class EmbeddingModel(BaseModel):
    """Model representing an embedding for content retrieval."""

    embedding_id: str = Field(..., description="Unique identifier for the embedding")
    content_id: str = Field(..., description="Reference to the original content")
    embedding: list[float] = Field(..., description="The actual embedding vector")
    model_name: str = Field(
        ..., description="Name of the model used to generate the embedding"
    )
    module: str = Field(default="", description="Module identifier")
    chapter: str = Field(default="", description="Chapter identifier")
    section: str = Field(default="", description="Section identifier")
    metadata: Optional[Dict[str, Any]] = Field(
        default_factory=dict, description="Additional metadata about the embedding"
    )
    created_at: datetime = Field(
        default_factory=datetime.utcnow, description="When the embedding was created"
    )
    updated_at: datetime = Field(
        default_factory=datetime.utcnow,
        description="When the embedding was last updated",
    )


class EmbeddingRequest(BaseModel):
    """Request model for embedding generation."""

    text: str = Field(
        ..., min_length=1, max_length=5000, description="Text to generate embedding for"
    )
    model_name: Optional[str] = Field(
        default="embed-english-v3.0", description="Name of the embedding model to use"
    )
    input_type: Optional[str] = Field(
        default="search_document", description="Type of input for the embedding model"
    )


class EmbeddingResponse(BaseModel):
    """Response model for embedding operations."""

    embedding_id: str = Field(
        ..., description="Unique identifier for the generated embedding"
    )
    content_id: str = Field(..., description="Reference to the original content")
    embedding: list[float] = Field(..., description="The generated embedding vector")
    model_name: str = Field(
        ..., description="Name of the model used to generate the embedding"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow, description="When the embedding was generated"
    )
