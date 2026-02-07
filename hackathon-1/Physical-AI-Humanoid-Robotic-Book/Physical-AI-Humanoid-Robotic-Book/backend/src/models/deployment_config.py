"""
Deployment Configuration Entity
Represents configuration parameters required for deploying the application components
"""

from dataclasses import dataclass
from datetime import datetime
from typing import Optional


@dataclass
class DeploymentConfiguration:
    """Configuration parameters required for deploying the application components"""

    id: str
    environment: str  # 'development' | 'staging' | 'production'
    backend_url: str  # URL for the backend API
    frontend_url: str  # URL for the frontend application
    database_url: str  # Connection string for Neon Postgres
    vector_database_url: str  # Connection string for Qdrant
    cohere_api_key: str  # API key for Cohere embeddings
    vercel_token: str  # Vercel deployment token
    created_at: datetime
    updated_at: datetime
    # Optional fields
    additional_secrets: Optional[dict] = None

    def __post_init__(self):
        """Validate the configuration after initialization"""
        if self.environment not in ["development", "staging", "production"]:
            raise ValueError(
                f"Environment must be one of 'development', 'staging', 'production', got '{self.environment}'"
            )

        # Validate URLs (basic validation)
        if not self.backend_url.startswith(("http://", "https://")):
            raise ValueError(
                f"backend_url must be a valid URL, got '{self.backend_url}'"
            )

        if not self.frontend_url.startswith(("http://", "https://")):
            raise ValueError(
                f"frontend_url must be a valid URL, got '{self.frontend_url}'"
            )

        if not self.database_url.startswith(("postgresql://", "postgres://")):
            raise ValueError(
                f"database_url must be a valid PostgreSQL URL, got '{self.database_url}'"
            )

        if not self.vector_database_url.startswith(("http://", "https://")):
            raise ValueError(
                f"vector_database_url must be a valid URL, got '{self.vector_database_url}'"
            )
