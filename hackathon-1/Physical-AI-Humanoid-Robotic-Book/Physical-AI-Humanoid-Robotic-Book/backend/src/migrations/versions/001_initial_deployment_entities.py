"""Initial migration for deployment entities

Revision ID: 001_initial_deployment_entities
Revises:
Create Date: 2025-12-17 05:09:54.206

"""

import sqlalchemy as sa
from alembic import op

# revision identifiers
revision = "001_initial_deployment_entities"
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    # Create deployment_configurations table
    op.create_table(
        "deployment_configurations",
        sa.Column("id", sa.String(), nullable=False),
        sa.Column("environment", sa.String(length=50), nullable=False),
        sa.Column("backend_url", sa.String(length=255), nullable=False),
        sa.Column("frontend_url", sa.String(length=255), nullable=False),
        sa.Column("webhook_secret", sa.String(length=255), nullable=False),
        sa.Column("is_active", sa.Boolean(), nullable=False, default=True),
        sa.Column("created_at", sa.DateTime(), nullable=False),
        sa.Column("updated_at", sa.DateTime(), nullable=False),
        sa.PrimaryKeyConstraint("id"),
    )

    # Create deployment_jobs table
    op.create_table(
        "deployment_jobs",
        sa.Column("id", sa.String(), nullable=False),
        sa.Column("job_type", sa.String(length=50), nullable=False),
        sa.Column("status", sa.String(length=20), nullable=False),
        sa.Column("start_time", sa.DateTime(), nullable=False),
        sa.Column("end_time", sa.DateTime(), nullable=True),
        sa.Column("commit_hash", sa.String(length=255), nullable=False),
        sa.Column("branch", sa.String(length=255), nullable=False),
        sa.Column("triggered_by", sa.String(length=255), nullable=False),
        sa.Column("error_message", sa.Text(), nullable=True),
        sa.PrimaryKeyConstraint("id"),
    )

    # Create deployment_artifacts table
    op.create_table(
        "deployment_artifacts",
        sa.Column("id", sa.String(), nullable=False),
        sa.Column("artifact_type", sa.String(length=50), nullable=False),
        sa.Column("storage_path", sa.String(length=255), nullable=False),
        sa.Column("size", sa.Integer(), nullable=False),
        sa.Column("checksum", sa.String(length=64), nullable=False),
        sa.Column("created_at", sa.DateTime(), nullable=False),
        sa.Column("expires_at", sa.DateTime(), nullable=False),
        sa.Column("job_id", sa.String(), nullable=False),
        sa.Column("additional_metadata", sa.Text(), nullable=True),
        sa.PrimaryKeyConstraint("id"),
    )

    # Create deployment_events table
    op.create_table(
        "deployment_events",
        sa.Column("id", sa.String(), nullable=False),
        sa.Column("event_type", sa.String(length=50), nullable=False),
        sa.Column("timestamp", sa.DateTime(), nullable=False),
        sa.Column("message", sa.Text(), nullable=False),
        sa.Column("metadata", sa.Text(), nullable=True),
        sa.Column("job_id", sa.String(), nullable=True),
        sa.Column("deployment_id", sa.String(), nullable=True),
        sa.PrimaryKeyConstraint("id"),
    )


def downgrade() -> None:
    # Drop deployment_events table
    op.drop_table("deployment_events")

    # Drop deployment_artifacts table
    op.drop_table("deployment_artifacts")

    # Drop deployment_jobs table
    op.drop_table("deployment_jobs")

    # Drop deployment_configurations table
    op.drop_table("deployment_configurations")
