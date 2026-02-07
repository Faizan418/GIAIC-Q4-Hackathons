"""Custom exceptions for the RAG Chatbot API."""

from typing import Optional

from fastapi import HTTPException, status


class RAGException(Exception):
    """Base exception class for RAG-related errors."""

    def __init__(self, message: str, error_code: Optional[str] = None):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class QueryProcessingError(RAGException):
    """Raised when there's an error processing a user query."""


class ContentRetrievalError(RAGException):
    """Raised when there's an error retrieving content from the vector store."""


class EmbeddingGenerationError(RAGException):
    """Raised when there's an error generating embeddings."""


class ExternalServiceError(RAGException):
    """Raised when external services (Cohere, OpenAI) are unavailable."""


class ValidationError(RAGException):
    """Raised when input validation fails."""


class ContentProcessingError(RAGException):
    """Raised when there's an error processing or indexing content."""


def create_http_exception(
    status_code: int, detail: str, headers: Optional[dict] = None
) -> HTTPException:
    """Helper function to create standardized HTTP exceptions."""
    return HTTPException(status_code=status_code, detail=detail, headers=headers)


def handle_rag_exception(exc: RAGException) -> HTTPException:
    """Convert RAG exceptions to appropriate HTTP exceptions."""
    if isinstance(exc, ValidationError):
        return create_http_exception(
            status_code=status.HTTP_400_BAD_REQUEST, detail=exc.message
        )
    elif isinstance(exc, ExternalServiceError):
        return create_http_exception(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"External service unavailable: {exc.message}",
        )
    else:
        return create_http_exception(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal error: {exc.message}",
        )
