"""
Deployment Event Entity
Log of deployment-related events for monitoring and auditing
"""

from dataclasses import dataclass
from datetime import datetime
from typing import Optional


@dataclass
class DeploymentEvent:
    """Log of deployment-related events for monitoring and auditing"""

    id: str
    event_type: str  # 'pipeline-started' | 'job-started' | 'job-completed' | 'job-failed' | 'deployment-success' | 'deployment-failure' | 'rollback-initiated' | 'rollback-completed'
    timestamp: datetime
    message: str
    # Optional fields
    metadata: Optional[dict] = None
    job_id: Optional[str] = None
    deployment_id: Optional[str] = None

    def __post_init__(self):
        """Validate the deployment event after initialization"""
        valid_event_types = [
            "pipeline-started",
            "job-started",
            "job-completed",
            "job-failed",
            "deployment-success",
            "deployment-failure",
            "rollback-initiated",
            "rollback-completed",
        ]

        if self.event_type not in valid_event_types:
            raise ValueError(
                f"Event type must be one of {valid_event_types}, got '{self.event_type}'"
            )

        # Validate timestamp is not in the future
        if self.timestamp > datetime.now():
            raise ValueError(
                f"Event timestamp cannot be in the future, got '{self.timestamp}'"
            )

        # Validate message is not empty
        if not self.message.strip():
            raise ValueError(f"Event message cannot be empty, got '{self.message}'")

        # Validate message length is reasonable
        if len(self.message) > 1000:
            raise ValueError(
                f"Event message is too long (max 1000 chars), got {len(self.message)} chars"
            )
