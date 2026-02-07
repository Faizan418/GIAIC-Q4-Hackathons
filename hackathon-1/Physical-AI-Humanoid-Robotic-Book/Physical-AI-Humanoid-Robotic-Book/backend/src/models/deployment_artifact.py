"""
Deployment Artifact Entity
Represents build artifacts generated during the deployment process
"""

from dataclasses import dataclass
from datetime import datetime
from typing import Optional


@dataclass
class DeploymentArtifact:
    """Build artifacts generated during the deployment process"""

    id: str
    artifact_type: str  # 'frontend-build' | 'backend-container' | 'embeddings-data'
    storage_path: str
    size: int  # Size in bytes
    checksum: str  # SHA256 checksum
    created_at: datetime
    expires_at: datetime
    job_id: str
    # Optional fields
    additional_metadata: Optional[dict] = None

    def __post_init__(self):
        """Validate the deployment artifact after initialization"""
        valid_artifact_types = [
            "frontend-build",
            "backend-container",
            "embeddings-data",
        ]
        if self.artifact_type not in valid_artifact_types:
            raise ValueError(
                f"Artifact type must be one of {valid_artifact_types}, got '{self.artifact_type}'"
            )

        # Validate checksum is a valid SHA256 (64 hexadecimal characters)
        if len(self.checksum) != 64 or not all(
            c in "0123456789abcdefABCDEF" for c in self.checksum
        ):
            raise ValueError(
                f"Checksum must be a valid SHA256 hash (64 hex chars), got '{self.checksum}'"
            )

        # Validate size is non-negative
        if self.size < 0:
            raise ValueError(f"Size must be non-negative, got '{self.size}'")

        # Validate created_at is not in the future
        if self.created_at > datetime.now():
            raise ValueError(
                f"Creation time cannot be in the future, got '{self.created_at}'"
            )

        # Validate expires_at is in the future
        if self.expires_at <= datetime.now():
            raise ValueError(
                f"Expiration time must be in the future, got '{self.expires_at}'"
            )

        # Validate expiration is after creation
        if self.expires_at <= self.created_at:
            raise ValueError(
                f"Expiration time must be after creation time, got created='{self.created_at}', expires='{self.expires_at}'"
            )
