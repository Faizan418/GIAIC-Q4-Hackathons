"""
Deployment Job Entity
Represents a single deployment job execution
"""

from dataclasses import dataclass
from datetime import datetime
from typing import Optional


@dataclass
class DeploymentJob:
    """Represents a single deployment job execution"""

    id: str
    job_type: str  # 'lint' | 'test' | 'build-frontend' | 'build-backend' | 'deploy-frontend' | 'deploy-backend' | 'migrate-db' | 'populate-embeddings'
    status: str  # 'pending' | 'running' | 'success' | 'failure' | 'cancelled'
    start_time: datetime
    commit_hash: str
    branch: str
    triggered_by: str
    # Optional fields
    end_time: Optional[datetime] = None
    logs: Optional[str] = None

    def __post_init__(self):
        """Validate the deployment job after initialization"""
        valid_job_types = [
            "lint",
            "test",
            "build-frontend",
            "build-backend",
            "deploy-frontend",
            "deploy-backend",
            "migrate-db",
            "populate-embeddings",
        ]

        if self.job_type not in valid_job_types:
            raise ValueError(
                f"Job type must be one of {valid_job_types}, got '{self.job_type}'"
            )

        valid_statuses = ["pending", "running", "success", "failure", "cancelled"]
        if self.status not in valid_statuses:
            raise ValueError(
                f"Status must be one of {valid_statuses}, got '{self.status}'"
            )

        # Validate commit hash format (basic validation - 7+ hex characters)
        if len(self.commit_hash) < 7:
            raise ValueError(
                f"Commit hash must be at least 7 characters, got '{self.commit_hash[:10]}...'"
            )

        # Validate start time is not in the future
        if self.start_time > datetime.now():
            raise ValueError(
                f"Start time cannot be in the future, got '{self.start_time}'"
            )

        # If end_time is provided, validate it's after start_time
        if self.end_time is not None and self.end_time < self.start_time:
            raise ValueError(
                f"End time must be after start time, got start='{self.start_time}', end='{self.end_time}'"
            )
