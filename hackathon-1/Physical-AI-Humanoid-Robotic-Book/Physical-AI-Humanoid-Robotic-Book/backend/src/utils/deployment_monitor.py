"""
Deployment Status Monitoring Utility
Utility functions for monitoring deployment status and health
"""

import asyncio
import logging
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional

from ..models.deployment_event import DeploymentEvent
from ..models.deployment_job import DeploymentJob


@dataclass
class DeploymentStatus:
    """Represents the current status of a deployment"""

    job_id: str
    status: str
    progress: int
    current_step: str
    start_time: datetime
    estimated_completion_time: Optional[datetime]
    logs: List[Dict[str, Any]]
    health_status: str  # 'healthy', 'warning', 'error'


class DeploymentMonitor:
    """
    Utility class for monitoring deployment status and health
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.active_monitors: Dict[str, asyncio.Task] = {}
        self.deployment_statuses: Dict[str, DeploymentStatus] = {}

    def calculate_progress(self, job: DeploymentJob) -> int:
        """
        Calculate progress percentage for a deployment job
        """
        progress_map = {
            "pending": 0,
            "running": 25,  # Initial running state
            "linting": 40,
            "testing": 60,
            "building": 80,
            "deploying": 90,
            "success": 100,
            "failure": 100,
            "cancelled": 100,
        }

        # In a real implementation, we would track the actual step the job is in
        # For now, we'll use the status to determine progress
        return progress_map.get(job.status, 0)

    def determine_current_step(self, job: DeploymentJob) -> str:
        """
        Determine the current step in the deployment process
        """
        status_to_step = {
            "pending": "Queued",
            "running": "Initializing",
            "linting": "Code Quality Checks",
            "testing": "Running Tests",
            "building": "Building Artifacts",
            "deploying": "Deploying",
            "success": "Completed",
            "failure": "Failed",
            "cancelled": "Cancelled",
        }

        return status_to_step.get(job.status, "Unknown")

    def estimate_completion_time(
        self, start_time: datetime, progress: int
    ) -> Optional[datetime]:
        """
        Estimate completion time based on progress
        """
        if progress == 0:
            # If no progress made yet, estimate 5 minutes from start
            return start_time + timedelta(minutes=5)

        if progress >= 100:
            return start_time

        # Estimate based on current progress rate
        elapsed = datetime.now() - start_time
        if progress > 0:
            total_estimated_time = elapsed * (100 / progress)
            estimated_completion = start_time + total_estimated_time
            return estimated_completion

        return start_time + timedelta(minutes=5)

    def generate_logs(
        self, job_id: str, events: List[DeploymentEvent]
    ) -> List[Dict[str, Any]]:
        """
        Generate log entries for a deployment job
        """
        logs = []

        # Filter events for this job
        job_events = [event for event in events if event.job_id == job_id]

        for event in job_events:
            log_entry = {
                "timestamp": event.timestamp.isoformat(),
                "level": (
                    "info"
                    if "success" in event.event_type or "completed" in event.event_type
                    else "error"
                    if "failure" in event.event_type or "failed" in event.event_type
                    else "info"
                ),
                "message": event.message,
            }
            logs.append(log_entry)

        return logs

    def get_health_status(self, job: DeploymentJob) -> str:
        """
        Determine health status based on job status and other factors
        """
        if job.status in ["failure", "cancelled"]:
            return "error"
        elif job.status in ["pending", "running"]:
            return "warning"  # In progress
        elif job.status == "success":
            return "healthy"
        else:
            return "warning"

    def get_deployment_status(
        self, job: DeploymentJob, events: List[DeploymentEvent]
    ) -> DeploymentStatus:
        """
        Get the current status of a deployment job
        """
        progress = self.calculate_progress(job)
        current_step = self.determine_current_step(job)
        estimated_completion = self.estimate_completion_time(job.start_time, progress)
        logs = self.generate_logs(job.id, events)
        health_status = self.get_health_status(job)

        status = DeploymentStatus(
            job_id=job.id,
            status=job.status,
            progress=progress,
            current_step=current_step,
            start_time=job.start_time,
            estimated_completion_time=estimated_completion,
            logs=logs,
            health_status=health_status,
        )

        return status

    async def monitor_deployment(self, job_id: str, update_callback=None):
        """
        Monitor a deployment job and update status periodically
        """
        self.logger.info(f"Starting to monitor deployment job: {job_id}")

        try:
            while True:
                # In a real implementation, this would check the actual job status
                # For now, we'll just sleep and pretend we're monitoring
                await asyncio.sleep(30)  # Check every 30 seconds

                # If the job is complete, stop monitoring
                # In a real implementation, we'd check the actual job status
                # For demo purposes, we'll just continue monitoring
                if update_callback:
                    await update_callback(job_id)

        except asyncio.CancelledError:
            self.logger.info(f"Monitoring cancelled for job: {job_id}")
        except Exception as e:
            self.logger.error(f"Error monitoring job {job_id}: {str(e)}")

    def start_monitoring(self, job_id: str, update_callback=None):
        """
        Start monitoring a deployment job
        """
        if job_id in self.active_monitors:
            self.logger.warning(f"Monitoring already active for job: {job_id}")
            return

        monitor_task = asyncio.create_task(
            self.monitor_deployment(job_id, update_callback)
        )
        self.active_monitors[job_id] = monitor_task
        self.logger.info(f"Started monitoring for job: {job_id}")

    def stop_monitoring(self, job_id: str):
        """
        Stop monitoring a deployment job
        """
        if job_id in self.active_monitors:
            monitor_task = self.active_monitors[job_id]
            monitor_task.cancel()
            del self.active_monitors[job_id]
            self.logger.info(f"Stopped monitoring for job: {job_id}")

    def get_all_monitored_jobs(self) -> List[str]:
        """
        Get list of all currently monitored job IDs
        """
        return list(self.active_monitors.keys())

    def cleanup_completed_jobs(self):
        """
        Remove completed jobs from monitoring
        """
        completed_jobs = []

        for job_id, task in self.active_monitors.items():
            if task.done():
                completed_jobs.append(job_id)

        for job_id in completed_jobs:
            del self.active_monitors[job_id]
            self.logger.info(f"Removed completed job from monitoring: {job_id}")


# Global instance for convenience
deployment_monitor = DeploymentMonitor()
