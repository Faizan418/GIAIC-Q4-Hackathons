"""Configuration management for the RAG Chatbot API."""


from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # API Keys
    cohere_api_key: str = "your_api_key"
    gemini_api_key: str = "your_api_key"

    # Database settings
    neon_postgres_url: str = "postgresql://neondb_owner:npg_lF7p6eSIxOQk@ep-wild-truth-a4b13qm7-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

    # Qdrant settings
    qdrant_url: str = (
        "c6399b27-5ad3-475b-a578-e8cbb5fa6e7d.europe-west3-0.gcp.cloud.qdrant.io:6333"
    )
    qdrant_api_key: str = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0._M-fZpqzPqgbD-U8NdFJcirPRgWOS-ofiiYXM3exCDs"

    # Server settings
    uvicorn_host: str = "0.0.0.0"
    uvicorn_port: int = 8000

    # Application settings
    app_name: str = "RAG Chatbot API"
    debug: bool = False
    max_query_length: int = 1000
    max_context_length: int = 2000
    max_concurrent_requests: int = 100

    # Cohere settings
    cohere_model: str = "embed-multilingual-v2.0"
    qdrant_collection: str = "Book-Embedding"

    # Authentication settings
    secret_key: str = "lECk7drIbqcAq04bcE6kQkQjw9qedV1s"
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False


# Global settings instance - will be initialized when get_settings is called
_settings_instance = None


def get_settings():
    """Get settings instance, loading from environment if needed."""
    global _settings_instance
    if _settings_instance is None:
        _settings_instance = Settings()
    return _settings_instance


# For backward compatibility, create the settings instance when needed
settings = get_settings()
