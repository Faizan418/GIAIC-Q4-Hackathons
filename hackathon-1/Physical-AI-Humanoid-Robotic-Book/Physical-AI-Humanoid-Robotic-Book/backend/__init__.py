"""Backend package for RAG Chatbot API."""

__version__ = "1.0.0"
__all__ = [
    "get_settings",
    "Settings",
    "settings",
    "get_app",
]

# Import commonly used items for convenient access
from backend.config import Settings, get_settings, settings


# Lazy import of app to avoid circular dependencies
def get_app():
    """Get the FastAPI application instance."""
    from backend.main import app

    return app
