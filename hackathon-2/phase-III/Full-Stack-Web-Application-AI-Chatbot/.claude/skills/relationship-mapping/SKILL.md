---
name: relationship-mapping
description: Properly defines one-to-one, one-to-many, many-to-many relationships in SQLModel with back_populates, cascade rules, and lazy/eager loading considerations.
---

# Relationship Mapping Skill

Clean relationships prevent common ORM pitfalls.

## Relationship Types

### 1. One-to-Many (Most Common)
```python
class User(BaseUUIDModel, table=True):
    posts: List["Post"] = Relationship(back_populates="author")

class Post(BaseUUIDModel, table=True):
    author_id: uuid.UUID = Field(foreign_key="user.id")
    author: User = Relationship(back_populates="posts")
2. One-to-One
Pythonclass User(BaseUUIDModel, table=True):
    profile: Optional["UserProfile"] = Relationship(
        back_populates="user",
        sa_relationship_kwargs={"uselist": False}
    )

class UserProfile(BaseUUIDModel, table=True):
    user_id: uuid.UUID = Field(foreign_key="user.id", unique=True)
    user: User = Relationship(back_populates="profile")
3. Many-to-Many
Pythonpost_tag = Table(
    "post_tag",
    Base.metadata,
    Column("post_id", ForeignKey("post.id"), primary_key=True),
    Column("tag_id", ForeignKey("tag.id"), primary_key=True)
)

class Post(BaseUUIDModel, table=True):
    tags: List["Tag"] = Relationship(back_populates="posts", link_model=post_tag)

class Tag(BaseUUIDModel, table=True):
    posts: List["Post"] = Relationship(back_populates="tags", link_model=post_tag)