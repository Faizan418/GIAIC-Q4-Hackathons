"""Main FastAPI application for the RAG Chatbot API."""

import logging
import os
import sys
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.responses import JSONResponse

# Add the project root to Python path to allow imports
project_root = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, project_root)

# Load environment variables first
from dotenv import load_dotenv

load_dotenv()

# Import config and get settings BEFORE importing other modules that depend on settings
from backend.config import get_settings

settings = get_settings()  # Get settings after environment is loaded

from api.analytics import router as analytics_router
from api.auth import router as auth_router
from api.conversation import router as conversation_router
from api.query import router as query_router
from api.text_selection import router as text_selection_router
from backend.db.neon_postgres import db
from services.rag_service import rag_service

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan events for the application."""
    logger.info("Starting up RAG Chatbot API")
    # Initialize the RAG service
    await rag_service.initialize()
    yield
    logger.info("Shutting down RAG Chatbot API")
    # Cleanup if needed
    await db.disconnect()


# Create FastAPI app with lifespan
app = FastAPI(
    title="RAG Chatbot API",
    description="API for Retrieval-Augmented Generation chatbot for Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    lifespan=lifespan,
)

# Include routers
app.include_router(query_router, prefix="/api", tags=["query"])
app.include_router(text_selection_router, prefix="/api", tags=["text-selection"])
app.include_router(conversation_router, prefix="", tags=["conversation"])
app.include_router(analytics_router, prefix="", tags=["analytics"])
app.include_router(auth_router, prefix="", tags=["auth"])


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "timestamp": "2025-12-16T10:00:00Z",
        "dependencies": {
            "cohere_api": "connected",
            "qdrant": "connected",
            "postgres": "connected",
        },
    }


@app.exception_handler(404)
async def not_found_handler(request, exc):
    """Handle 404 errors."""
    return JSONResponse(
        status_code=404,
        content={
            "error": "Endpoint not found",
            "message": "The requested endpoint does not exist",
        },
    )


@app.exception_handler(500)
async def internal_error_handler(request, exc):
    """Handle 500 errors."""
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal server error",
            "message": "An unexpected error occurred",
        },
    )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app", host=settings.uvicorn_host, port=settings.uvicorn_port, reload=True
    )
