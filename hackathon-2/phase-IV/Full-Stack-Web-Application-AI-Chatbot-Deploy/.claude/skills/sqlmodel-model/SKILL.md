---
name: sqlmodel-model
description: Creates advanced SQLModel models with relationships, indexes, constraints, enums, JSONB fields, computed columns, and migration-friendly patterns.
---

# SQLModel Model Skill

Advanced modeling techniques for real-world applications.

## Advanced Features

### 1. Enums & Constraints
```python
from enum import Enum
from sqlmodel import Field

class UserRole(str, Enum):
    USER = "user"
    ADMIN = "admin"
    MODERATOR = "moderator"

class User(SQLModel, table=True):
    role: UserRole = Field(default=UserRole.USER)
2. Composite Indexes & Partial Indexes
Pythonfrom sqlmodel import Index, Column
import sqlalchemy as sa

class Post(SQLModel, table=True):
    __table_args__ = (
        Index("idx_posts_author_published", "author_id", "published_at", 
              postgresql_where=sa.text("published_at IS NOT NULL")),
    )
3. JSONB with Indexing
Pythonmetadata: dict = Field(default={}, sa_column=Column(JSONB))
4. Full Featured Example
Pythonclass User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    email: str = Field(unique=True, index=True)
    preferences: dict = Field(default={}, sa_column=Column(JSONB))
    
    posts: List["Post"] = Relationship(back_populates="author")
    
    __table_args__ = (
        Index("idx_users_preferences_theme", 
              sa.text("(preferences->>'theme')"), 
              postgresql_using="gin"),
    )