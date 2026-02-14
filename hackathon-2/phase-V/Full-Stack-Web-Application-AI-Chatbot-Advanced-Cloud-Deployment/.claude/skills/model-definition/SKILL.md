---
name: model-definition
description: Defines clean, reusable SQLModel models with proper field types, defaults, constraints, indexes, and separation of concerns between table models and schemas.
---

# Model Definition Skill

This skill creates professional, type-safe SQLModel models following 2026 best practices.

## Best Practices
- One file per model or group related models
- Use SQLModel for table definitions
- Separate Pydantic schemas for input/output
- Explicit field definitions
- Proper use of defaults, nullable, indexes
- Table args for complex constraints

## Recommended Structure
app/
├── models/
│   ├── init.py
│   ├── base.py            # Common mixins
│   ├── user.py
│   ├── post.py
│   └── item.py
├── schemas/
│   ├── user.py
│   └── post.py
└── crud/
text## Advanced Model Example
```python
from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List
from datetime import datetime
import uuid

class BaseUUIDModel(SQLModel):
    id: uuid.UUID = Field(
        default_factory=uuid.uuid4,
        primary_key=True,
        index=True,
        nullable=False
    )

class User(BaseUUIDModel, table=True):
    email: str = Field(unique=True, index=True, max_length=255)
    hashed_password: str
    full_name: Optional[str] = Field(default=None, max_length=100)
    is_active: bool = Field(default=True)
    is_superuser: bool = Field(default=False)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    
    posts: List["Post"] = Relationship(back_populates="author")
    
    __table_args__ = (
        Index("ix_user_email", "email"),
    )