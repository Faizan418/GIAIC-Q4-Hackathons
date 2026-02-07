from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime
import uuid
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
import sqlalchemy as sa


class User(SQLModel, table=True):
    """
    User entity representing a registered user in the system
    """
    id: str = Field(
        sa_column=sa.Column(
            PostgresUUID(as_uuid=True),
            primary_key=True,
            default=uuid.uuid4
        )
    )
    email: str = Field(sa_column=sa.Column(sa.String, unique=True, nullable=False))
    name: Optional[str] = Field(sa_column=sa.Column(sa.String(100)))  # Optional name field
    password_hash: str = Field(sa_column=sa.Column(sa.String, nullable=False))  # For authentication
    created_at: datetime = Field(default_factory=datetime.utcnow, sa_column=sa.Column(sa.DateTime, nullable=False))
    updated_at: datetime = Field(default_factory=datetime.utcnow, sa_column=sa.Column(sa.DateTime, nullable=False))
    is_active: bool = Field(default=True, sa_column=sa.Column(sa.Boolean, nullable=False))