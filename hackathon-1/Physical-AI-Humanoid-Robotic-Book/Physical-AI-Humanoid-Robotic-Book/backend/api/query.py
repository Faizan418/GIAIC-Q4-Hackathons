"""Query API endpoints for the RAG Chatbot API."""

import logging
import time

from fastapi import APIRouter, HTTPException, status

from backend.models.query import QueryRequest
from backend.models.response import QueryResponse
from backend.services.rag_service import rag_service
from backend.utils.exceptions import ValidationError, handle_rag_exception
from backend.utils.logging import rag_logger

router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/query", response_model=QueryResponse, tags=["query"])
async def query_endpoint(query_request: QueryRequest):
    """Handle user queries and return relevant responses based on textbook content."""
    start_time = time.time()

    try:
        # Validate query length
        if len(query_request.query) > 1000:
            raise ValidationError("Query must be 1000 characters or less")

        if not query_request.query.strip():
            raise ValidationError("Query cannot be empty or whitespace only")

        # Process the query through RAG service
        response = await rag_service.process_query(query_request)

        # Log the API call with timing
        duration = time.time() - start_time
        rag_logger.log_api_call("/api/query", "POST", duration, 200)

        return response

    except ValidationError as e:
        # Log validation errors
        duration = time.time() - start_time
        rag_logger.log_api_call("/api/query", "POST", duration, 400)
        logger.warning(f"Validation error in query endpoint: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "Validation Error", "message": e.message},
        )
    except Exception as e:
        # Handle other exceptions
        duration = time.time() - start_time
        rag_logger.log_api_call("/api/query", "POST", duration, 500)

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

        logger.error(f"Error in query endpoint: {str(http_exc.detail)}")
        raise http_exc


@router.get("/query/health", tags=["query"])
async def query_health():
    """Health check for the query endpoint."""
    return {
        "status": "healthy",
        "endpoint": "/api/query",
        "timestamp": "2025-12-16T10:00:00Z",
    }
