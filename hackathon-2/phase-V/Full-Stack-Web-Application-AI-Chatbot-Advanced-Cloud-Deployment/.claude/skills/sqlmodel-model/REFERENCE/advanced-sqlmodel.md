# Advanced SQLModel Patterns

## Soft Delete
```python
class BaseModel(SQLModel):
    deleted_at: Optional[datetime] = Field(default=None)

class Post(BaseModel, table=True):
    # queries should filter deleted_at IS NULL
Audit Trail
Pythonclass AuditMixin(SQLModel):
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow, 
                                sa_column_kwargs={"onupdate": datetime.utcnow})

class User(AuditMixin, table=True):
    ...
Full Text Search
Pythonfrom sqlalchemy import text

class Post(SQLModel, table=True):
    __table_args__ = (
        Index("idx_posts_fts", text("to_tsvector('english', title || ' ' || content)"), postgresql_using="gin"),
    )