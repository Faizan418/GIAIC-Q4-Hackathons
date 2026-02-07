#!/usr/bin/env python3
"""Test script to debug environment variable loading."""

import os

from dotenv import load_dotenv

# Load environment variables
dotenv_path = os.path.join(os.path.dirname(__file__), ".env")
load_dotenv(dotenv_path)

print("Environment variables loaded from .env file:")
print(f"COHERE_API_KEY: {os.getenv('COHERE_API_KEY', 'NOT FOUND')[:10]}...")
print(f"GEMINI_API_KEY: {os.getenv('GEMINI_API_KEY', 'NOT FOUND')}")
print(f"NEON_POSTGRES_URL: {os.getenv('NEON_POSTGRES_URL', 'NOT FOUND')[:50]}...")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL', 'NOT FOUND')}")
print(f"QDRANT_API_KEY: {os.getenv('QDRANT_API_KEY', 'NOT FOUND')[:10]}...")

# Now try to import and create settings
try:
    from backend.config import get_settings

    print("\nTrying to create Settings instance...")
    settings = get_settings()
    print("SUCCESS: Settings created successfully!")
    print(f"Cohere API Key: {settings.cohere_api_key[:10]}...")
    print(f"Database URL: {settings.neon_postgres_url[:50]}...")
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Host: {settings.uvicorn_host}")
    print(f"Port: {settings.uvicorn_port}")
except Exception as e:
    print(f"ERROR creating settings: {e}")
    import traceback

    traceback.print_exc()
