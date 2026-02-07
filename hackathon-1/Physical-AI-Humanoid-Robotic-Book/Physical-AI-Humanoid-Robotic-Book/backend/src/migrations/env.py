from logging.config import fileConfig

from alembic import context
from sqlalchemy import engine_from_config, pool

# Import our models for the migration

# this is the Alembic Config object, which provides
# access to the values within the .ini file in use.
config = context.config

# Interpret the config file for Python logging.
# This line sets up loggers basically.
if config.config_file_name is not None:
    fileConfig(config.config_file_name)

# add your model's MetaData object here for 'autogenerate' support
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

import uuid
from datetime import datetime

# For our models to be discovered by alembic, we need to map them to SQLAlchemy tables
# Since our models are dataclasses, we'll create a mapping here
from sqlalchemy import Boolean, Column, DateTime, Integer, String, Text

# Import all models to include them in the metadata


# Define equivalent SQLAlchemy tables for our dataclasses
class DeploymentConfigurationTable(Base):
    __tablename__ = "deployment_configurations"

    id = Column(
        String, primary_key=True, default=lambda: f"cfg_{str(uuid.uuid4())[:8]}"
    )
    environment = Column(String(50), nullable=False)
    backend_url = Column(String(255), nullable=False)
    frontend_url = Column(String(255), nullable=False)
    webhook_secret = Column(String(255), nullable=False)
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime, default=datetime.now)
    updated_at = Column(DateTime, default=datetime.now, onupdate=datetime.now)


class DeploymentJobTable(Base):
    __tablename__ = "deployment_jobs"

    id = Column(String, primary_key=True)
    job_type = Column(String(50), nullable=False)
    status = Column(String(20), nullable=False)
    start_time = Column(DateTime, nullable=False)
    end_time = Column(DateTime)
    commit_hash = Column(String(255), nullable=False)
    branch = Column(String(255), nullable=False)
    triggered_by = Column(String(255), nullable=False)
    error_message = Column(Text)


class DeploymentArtifactTable(Base):
    __tablename__ = "deployment_artifacts"

    id = Column(String, primary_key=True)
    artifact_type = Column(String(50), nullable=False)
    storage_path = Column(String(255), nullable=False)
    size = Column(Integer, nullable=False)
    checksum = Column(String(64), nullable=False)
    created_at = Column(DateTime, nullable=False)
    expires_at = Column(DateTime, nullable=False)
    job_id = Column(String, nullable=False)
    additional_metadata = Column(Text)  # JSON data as text


class DeploymentEventTable(Base):
    __tablename__ = "deployment_events"

    id = Column(String, primary_key=True)
    event_type = Column(String(50), nullable=False)
    timestamp = Column(DateTime, nullable=False)
    message = Column(Text, nullable=False)
    metadata = Column(Text)  # JSON data as text
    job_id = Column(String)
    deployment_id = Column(String)


target_metadata = Base.metadata


def run_migrations_offline() -> None:
    """Run migrations in 'offline' mode.

    This configures the context with just a URL
    and not an Engine, though an Engine is acceptable
    here as well.  By skipping the Engine creation
    we don't even need a DBAPI to be available.

    Calls to context.execute() here emit the given string to the
    script output.

    """
    url = config.get_main_option("sqlalchemy.url")
    context.configure(
        url=url,
        target_metadata=target_metadata,
        literal_binds=True,
        dialect_opts={"paramstyle": "named"},
    )

    with context.begin_transaction():
        context.run_migrations()


def run_migrations_online() -> None:
    """Run migrations in 'online' mode.

    In this scenario we need to create an Engine
    and associate a connection with the context.

    """
    connectable = engine_from_config(
        config.get_section(config.config_ini_section) or {},
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )

    with connectable.connect() as connection:
        context.configure(connection=connection, target_metadata=target_metadata)

        with context.begin_transaction():
            context.run_migrations()


if context.is_offline_mode():
    run_migrations_offline()
else:
    run_migrations_online()
