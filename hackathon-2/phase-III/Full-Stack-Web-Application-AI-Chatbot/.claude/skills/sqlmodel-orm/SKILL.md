---
name: sqlmodel-orm
description: Defines database models using SQLModel (SQLAlchemy + Pydantic) for type-safe tables, relationships, and seamless integration with FastAPI schemas.
---

# SQLModel ORM Skill

SQLModel (created by FastAPI author) is the recommended ORM for new FastAPI projects in 2026 – combines SQLAlchemy power with Pydantic simplicity.

## Why SQLModel
- Single model for DB table + Pydantic validation
- Excellent type hints and editor support
- Perfect FastAPI integration
- Built on mature SQLAlchemy core

## Model Structure

```python
from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List

class User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    email: str = Field(index=True, unique=True)
    hashed_password: str
    is_active: bool = True
    
    # Relationships
    items: List["Item"] = Relationship(back_populates="owner")

class Item(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    title: str
    description: Optional[str] = None
    owner_id: int = Field(foreign_key="user.id")
    
    owner: User = Relationship(back_populates="items")
Engine & Session Setup
Pythonfrom sqlmodel import create_engine, Session

engine = create_engine("postgresql://user:pass@localhost/db")

def get_session():
    with Session(engine) as session:
        yield session