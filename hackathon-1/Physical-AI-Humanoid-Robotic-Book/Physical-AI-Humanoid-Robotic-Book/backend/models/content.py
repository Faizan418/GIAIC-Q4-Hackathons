"""Content model for the RAG Chatbot API."""

from datetime import datetime
from typing import Any, Dict, Optional

from pydantic import BaseModel, Field


class TextbookContent(BaseModel):
    """Model representing structured educational material from the Physical AI & Humanoid Robotics textbook."""

    content_id: str = Field(..., description="Unique identifier for the content piece")
    title: str = Field(..., description="Title of the content section")
    text: str = Field(..., description="The actual content text")
    module: str = Field(
        ..., description="Module identifier (e.g., '001-intro-physical-ai')"
    )
    chapter: str = Field(..., description="Chapter identifier within the module")
    section: str = Field(..., description="Section identifier within the chapter")
    page_numbers: Optional[str] = Field(None, description="Page range in the textbook")
    metadata: Optional[Dict[str, Any]] = Field(
        default_factory=dict, description="Additional metadata about the content"
    )
    created_at: datetime = Field(
        default_factory=datetime.utcnow, description="When the content was created"
    )
    updated_at: datetime = Field(
        default_factory=datetime.utcnow, description="When the content was last updated"
    )


class ContentChunk(BaseModel):
    """Model representing a chunk of content for embedding."""

    content_id: str = Field(..., description="Reference to the original content")
    text_chunk: str = Field(..., description="The text that was embedded")
    chunk_index: int = Field(
        ..., description="Position of this chunk in the original content"
    )
    module: str = Field(default="", description="Module identifier")
    chapter: str = Field(default="", description="Chapter identifier")
    section: str = Field(default="", description="Section identifier")
