# Model Definition Templates

## Timestamp Mixin
```python
from datetime import datetime
from sqlmodel import Field

class TimestampModel(SQLModel):
    created_at: datetime = Field(default_factory=datetime.utcnow, nullable=False)
    updated_at: datetime = Field(
        default_factory=datetime.utcnow,
        nullable=False,
        sa_column_kwargs={"onupdate": datetime.utcnow}
    )
Soft Delete Mixin
Pythonclass SoftDeleteModel(SQLModel):
    is_deleted: bool = Field(default=False)
    deleted_at: Optional[datetime] = Field(default=None)
Full Featured User Model
Pythonclass User(BaseUUIDModel, TimestampModel, table=True):
    email: str = Field(..., unique=True, index=True)
    hashed_password: str = Field(..., max_length=255)
    is_active: bool = True
    role: str = Field(default="user", max_length=20)
    
    profile: Optional["UserProfile"] = Relationship(back_populates="user")