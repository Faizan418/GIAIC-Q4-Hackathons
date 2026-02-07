"""
Deployment API Endpoints
REST API endpoints for deployment management and monitoring
"""

import uuid
from datetime import datetime
from typing import List, Optional

from fastapi import APIRouter, HTTPException

from ..models.deployment_artifact import DeploymentArtifact
from ..models.deployment_config import DeploymentConfiguration
from ..models.deployment_event import DeploymentEvent
from ..models.deployment_job import DeploymentJob

router = APIRouter(prefix="/api/deployment", tags=["deployment"])

# In-memory storage for demonstration purposes
# In a real application, these would be stored in a database
deployments: List[DeploymentJob] = []
configs: List[DeploymentConfiguration] = []
artifacts: List[DeploymentArtifact] = []
events: List[DeploymentEvent] = []


@router.get("/status/{deployment_id}")
async def get_deployment_status(deployment_id: str):
    """
    Get the current status and progress of a deployment
    """
    deployment = next((d for d in deployments if d.id == deployment_id), None)

    if not deployment:
        raise HTTPException(status_code=404, detail="Deployment not found")

    # Calculate progress based on status
    progress_map = {
        "pending": 0,
        "running": 50,
        "success": 100,
        "failure": 100,
        "cancelled": 100,
    }

    progress = progress_map.get(deployment.status, 0)

    # Mock logs for demonstration
    mock_logs = [
        {
            "timestamp": deployment.start_time.isoformat(),
            "level": "info",
            "message": f"Deployment {deployment_id} started",
        },
        {
            "timestamp": datetime.now().isoformat(),
            "level": "info",
            "message": "Processing deployment...",
        },
    ]

    if deployment.status in ["success", "failure", "cancelled"]:
        mock_logs.append(
            {
                "timestamp": datetime.now().isoformat(),
                "level": "info" if deployment.status == "success" else "error",
                "message": f"Deployment {deployment.status}",
            }
        )

    return {
        "deploymentId": deployment.id,
        "status": deployment.status,
        "progress": progress,
        "currentStep": deployment.job_type,
        "startTime": deployment.start_time.isoformat(),
        "estimatedCompletionTime": (
            deployment.start_time.timestamp() + 300
        ),  # 5 minutes from start
        "logs": mock_logs,
    }


@router.post("/trigger")
async def trigger_deployment(
    branch: str,
    environment: str,
    commit_hash: str,
    include_migrations: bool = True,
    include_embedding_update: bool = False,
):
    """
    Trigger a new deployment process
    """
    # Validate environment
    if environment not in ["development", "staging", "production"]:
        raise HTTPException(
            status_code=400, detail=f"Invalid environment: {environment}"
        )

    # Create a new deployment job
    job_id = f"job_{uuid.uuid4().hex[:8]}"
    new_job = DeploymentJob(
        id=job_id,
        job_type="deploy-backend",  # Simplified for demo
        status="pending",
        start_time=datetime.now(),
        commit_hash=commit_hash,
        branch=branch,
        triggered_by="api_call",  # In reality, this would come from auth
    )

    deployments.append(new_job)

    return {
        "deploymentId": job_id,
        "status": "pending",
        "message": "Deployment triggered successfully",
    }


@router.post("/rollback/{deployment_id}")
async def rollback_deployment(deployment_id: str, target_deployment_id: str):
    """
    Initiate rollback to a previous deployment state
    """
    # Check if the deployment exists
    deployment = next((d for d in deployments if d.id == deployment_id), None)
    if not deployment:
        raise HTTPException(status_code=404, detail="Deployment not found")

    # Create a rollback job
    rollback_job_id = f"rollback_{uuid.uuid4().hex[:8]}"
    rollback_job = DeploymentJob(
        id=rollback_job_id,
        job_type="rollback",
        status="pending",
        start_time=datetime.now(),
        commit_hash=target_deployment_id,  # Using target ID as commit hash for demo
        branch=deployment.branch,
        triggered_by="api_call",  # In reality, this would come from auth
    )

    deployments.append(rollback_job)

    return {
        "deploymentId": rollback_job_id,
        "status": "pending",
        "message": "Rollback initiated successfully",
    }


@router.get("/history")
async def get_deployment_history(
    environment: Optional[str] = None, limit: int = 10, offset: int = 0
):
    """
    Retrieve history of past deployments
    """
    # Filter deployments by environment if specified
    filtered_deployments = deployments
    if environment:
        if environment not in ["development", "staging", "production"]:
            raise HTTPException(
                status_code=400, detail=f"Invalid environment: {environment}"
            )
        filtered_deployments = [
            d for d in deployments if d.branch.startswith(environment)
        ]

    # Apply pagination
    paginated_deployments = filtered_deployments[offset : offset + limit]

    # Convert to response format
    deployment_list = []
    for dep in paginated_deployments:
        deployment_list.append(
            {
                "id": dep.id,
                "status": "success" if dep.status == "success" else "failure",
                "environment": dep.branch.split("-")[0]
                if "-" in dep.branch
                else "development",
                "commitHash": dep.commit_hash,
                "branch": dep.branch,
                "startTime": dep.start_time.isoformat(),
                "endTime": datetime.now().isoformat(),  # Mock end time
                "duration": 300,  # Mock duration in seconds
                "triggeredBy": dep.triggered_by,
            }
        )

    return {"deployments": deployment_list, "total": len(filtered_deployments)}
