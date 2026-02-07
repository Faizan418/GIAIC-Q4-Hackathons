"""Text Selection API endpoints for the RAG Chatbot API."""

import logging
import time
from typing import Optional

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field

from backend.models.response import QueryResponse
from backend.services.rag_service import rag_service
from backend.utils.exceptions import ValidationError, handle_rag_exception
from backend.utils.logging import rag_logger

router = APIRouter()
logger = logging.getLogger(__name__)


class TextSelectionRequestModel(BaseModel):
    """Request model for text selection query endpoint."""

    selected_text: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The selected text from the textbook",
    )
    context: Optional[str] = Field(
        None,
        max_length=2000,
        description="Additional context provided with the selection",
    )
    user_id: Optional[str] = Field(
        None, description="Identifier for the user making the selection"
    )
    conversation_id: Optional[str] = Field(
        None,
        description="ID of the conversation thread if continuing existing conversation",
    )


@router.post(
    "/text-selection-query", response_model=QueryResponse, tags=["text-selection"]
)
async def text_selection_query_endpoint(request: TextSelectionRequestModel):
    """Handle text selection queries and return contextual information."""
    start_time = time.time()

    try:
        # Process the text selection query through RAG service
        response = await rag_service.process_text_selection_query(
            request.selected_text,
            request.context,
            request.user_id,
            request.conversation_id,
        )

        # Log the API call with timing
        duration = time.time() - start_time
        rag_logger.log_api_call("/api/text-selection-query", "POST", duration, 200)

        return response

    except ValidationError as e:
        # Log validation errors
        duration = time.time() - start_time
        rag_logger.log_api_call("/api/text-selection-query", "POST", duration, 400)
        logger.warning(
            f"Validation error in text selection query endpoint: {e.message}"
        )
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "Validation Error", "message": e.message},
        )
    except Exception as e:
        # Handle other exceptions
        duration = time.time() - start_time
        rag_logger.log_api_call("/api/text-selection-query", "POST", duration, 500)

        http_exc = (
            handle_rag_exception(e)
            if hasattr(e, "message")
            else HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail={
                    "error": "Internal Server Error",
                    "message": "An unexpected error occurred",
                },
            )
        )

        logger.error(f"Error in text selection query endpoint: {str(http_exc.detail)}")
        raise http_exc


@router.get("/text-selection/health", tags=["text-selection"])
async def text_selection_health():
    """Health check for the text selection endpoint."""
    return {
        "status": "healthy",
        "endpoint": "/api/text-selection-query",
        "timestamp": "2025-12-16T10:00:00Z",
    }
