#!/usr/bin/env python3
"""Script to start the backend server with proper environment loading."""

import os
import sys

from dotenv import load_dotenv

# Load environment variables from .env file FIRST before importing anything else
dotenv_path = os.path.join(os.path.dirname(__file__), ".env")
load_dotenv(dotenv_path)

# Manually set environment variables to ensure they're available
with open(dotenv_path, "r") as f:
    for line in f:
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip()
            # Remove quotes if present
            if value.startswith(('"', "'")) and value.endswith(('"', "'")):
                value = value[1:-1]
            os.environ[key] = value

# Add the project root to the Python path so imports work correctly
project_root = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, project_root)

# Import and run the main application
if __name__ == "__main__":
    import uvicorn

    # Import after environment is loaded
    from backend.config import get_settings

    try:
        # Get settings after environment is loaded
        settings = get_settings()

        # Get host and port from settings or environment
        host = getattr(settings, "uvicorn_host", os.getenv("UVICORN_HOST", "0.0.0.0"))
        port = getattr(settings, "uvicorn_port", int(os.getenv("UVICORN_PORT", "8000")))

        print(f"Starting RAG Chatbot API server on {host}:{port}")
        print("Environment variables loaded successfully")

        # Run the application
        uvicorn.run(
            "backend.main:app",
            host=host,
            port=port,
            reload=False,
        )
    except Exception as e:
        print(f"Error starting server: {e}")
        print("Make sure all required environment variables are set in the .env file")
        sys.exit(1)
