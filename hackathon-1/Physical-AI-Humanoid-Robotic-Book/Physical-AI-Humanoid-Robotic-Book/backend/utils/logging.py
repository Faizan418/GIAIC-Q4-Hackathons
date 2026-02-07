"""Logging utilities for the RAG Chatbot API."""

import logging
from typing import Optional


class RAGLogger:
    """Custom logger for RAG-specific operations."""

    def __init__(self, name: str = "rag_logger"):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.INFO)

        # Create handler if not already configured
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

    def log_query(self, query_id: str, query_text: str, user_id: Optional[str] = None):
        """Log query information."""
        self.logger.info(
            f"Query received - ID: {query_id}, User: {user_id or 'unknown'}, Text: {query_text[:100]}..."
        )

    def log_response(self, response_id: str, query_id: str, confidence_score: float):
        """Log response information."""
        self.logger.info(
            f"Response generated - ID: {response_id}, Query: {query_id}, Confidence: {confidence_score}"
        )

    def log_embedding_process(self, content_id: str, status: str):
        """Log embedding process information."""
        self.logger.info(f"Embedding process - Content: {content_id}, Status: {status}")

    def log_error(
        self, error_type: str, error_message: str, query_id: Optional[str] = None
    ):
        """Log error information."""
        query_info = f", Query: {query_id}" if query_id else ""
        self.logger.error(
            f"Error - Type: {error_type}, Message: {error_message}{query_info}"
        )

    def log_api_call(
        self, endpoint: str, method: str, duration: float, status_code: int
    ):
        """Log API call information."""
        self.logger.info(
            f"API Call - {method} {endpoint}, Duration: {duration:.3f}s, Status: {status_code}"
        )


# Create a global instance
rag_logger = RAGLogger()
