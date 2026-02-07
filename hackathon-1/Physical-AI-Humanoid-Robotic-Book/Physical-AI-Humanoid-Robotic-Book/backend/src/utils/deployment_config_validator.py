"""
Deployment Configuration Validator
Utility for validating deployment configuration parameters
"""

import re
import secrets
import string
from typing import Any, Dict, List
from urllib.parse import urlparse


class DeploymentConfigValidator:
    """
    Utility class for validating deployment configuration parameters
    """

    @staticmethod
    def validate_environment(environment: str) -> bool:
        """
        Validate environment name
        """
        if not environment or not isinstance(environment, str):
            return False

        # Environment should be alphanumeric with hyphens and underscores, 2-50 chars
        pattern = r"^[a-zA-Z][a-zA-Z0-9_-]{1,49}$"
        return bool(re.match(pattern, environment))

    @staticmethod
    def validate_url(url: str) -> bool:
        """
        Validate URL format
        """
        if not url or not isinstance(url, str):
            return False

        try:
            result = urlparse(url)
            return all([result.scheme, result.netloc])
        except Exception:
            return False

    @staticmethod
    def validate_webhook_secret(secret: str) -> bool:
        """
        Validate webhook secret format
        """
        if not secret or not isinstance(secret, str):
            return False

        # Should be at least 16 characters for security
        return len(secret) >= 16

    @staticmethod
    def validate_deployment_config(config: Dict[str, Any]) -> Dict[str, List[str]]:
        """
        Validate a complete deployment configuration
        Returns a dictionary with validation results and any errors
        """
        errors = []

        # Validate environment
        environment = config.get("environment")
        if environment is None:
            errors.append("Environment is required")
        elif not DeploymentConfigValidator.validate_environment(environment):
            errors.append(
                f"Invalid environment: '{environment}'. Must be alphanumeric with hyphens/underscores, 2-50 chars, starting with a letter."
            )

        # Validate backend URL
        backend_url = config.get("backend_url")
        if backend_url is None:
            errors.append("Backend URL is required")
        elif not DeploymentConfigValidator.validate_url(backend_url):
            errors.append(f"Invalid backend URL: '{backend_url}'")

        # Validate frontend URL
        frontend_url = config.get("frontend_url")
        if frontend_url is None:
            errors.append("Frontend URL is required")
        elif not DeploymentConfigValidator.validate_url(frontend_url):
            errors.append(f"Invalid frontend URL: '{frontend_url}'")

        # Validate webhook secret
        webhook_secret = config.get("webhook_secret")
        if webhook_secret is None:
            errors.append("Webhook secret is required")
        elif not DeploymentConfigValidator.validate_webhook_secret(webhook_secret):
            errors.append("Webhook secret must be at least 16 characters long")

        # Check for any extra fields that shouldn't be there
        valid_fields = {
            "environment",
            "backend_url",
            "frontend_url",
            "webhook_secret",
            "is_active",
            "created_at",
            "updated_at",
            "id",
        }
        extra_fields = set(config.keys()) - valid_fields
        if extra_fields:
            errors.append(
                f"Unexpected fields in configuration: {', '.join(extra_fields)}"
            )

        # Validate is_active if present
        is_active = config.get("is_active")
        if is_active is not None and not isinstance(is_active, bool):
            errors.append("is_active must be a boolean value")

        return {"is_valid": len(errors) == 0, "errors": errors}

    @staticmethod
    def validate_branch_name(branch: str) -> bool:
        """
        Validate git branch name
        """
        if not branch or not isinstance(branch, str):
            return False

        # Git branch name validation (simplified)
        # Should not contain certain characters and should be reasonable length
        invalid_chars = set('<>:"|?*\\')
        if any(c in invalid_chars for c in branch):
            return False

        # Should not start or end with slash
        if branch.startswith("/") or branch.endswith("/"):
            return False

        # Should not contain double dots
        if ".." in branch:
            return False

        # Reasonable length
        return 1 <= len(branch) <= 100

    @staticmethod
    def validate_commit_hash(commit_hash: str) -> bool:
        """
        Validate git commit hash (40-character hexadecimal)
        """
        if not commit_hash or not isinstance(commit_hash, str):
            return False

        # Git commit hashes are typically 40 characters of hexadecimal
        if len(commit_hash) != 40:
            # Allow for shorter hashes (abbreviated)
            if len(commit_hash) < 7:
                return False
            # But they should still be hexadecimal
            return bool(re.match(r"^[a-fA-F0-9]+$", commit_hash))

        return bool(re.match(r"^[a-fA-F0-9]{40}$", commit_hash))

    @staticmethod
    def generate_secure_webhook_secret(length: int = 32) -> str:
        """
        Generate a secure webhook secret
        """
        alphabet = string.ascii_letters + string.digits
        return "".join(secrets.choice(alphabet) for _ in range(length))

    @staticmethod
    def validate_artifact_type(artifact_type: str) -> bool:
        """
        Validate deployment artifact type
        """
        valid_types = ["frontend-build", "backend-container", "embeddings-data"]
        return artifact_type in valid_types

    @staticmethod
    def validate_job_type(job_type: str) -> bool:
        """
        Validate deployment job type
        """
        valid_types = [
            "deploy-backend",
            "deploy-frontend",
            "rollback",
            "migrate-database",
            "populate-embeddings",
        ]
        return job_type in valid_types

    @staticmethod
    def validate_job_status(status: str) -> bool:
        """
        Validate deployment job status
        """
        valid_statuses = [
            "pending",
            "running",
            "linting",
            "testing",
            "building",
            "deploying",
            "success",
            "failure",
            "cancelled",
        ]
        return status in valid_statuses


# Global instance for convenience
config_validator = DeploymentConfigValidator()
