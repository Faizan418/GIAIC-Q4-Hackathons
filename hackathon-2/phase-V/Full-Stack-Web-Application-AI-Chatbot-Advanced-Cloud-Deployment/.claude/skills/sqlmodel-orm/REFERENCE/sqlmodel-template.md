# SQLModel Best Practices

## Hero Model Example (User with Posts)
```python
class User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    email: str = Field(unique=True)
    
    posts: List["Post"] = Relationship(back_populates="author")

class Post(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    title: str
    content: str
    author_id: int = Field(foreign_key="user.id")
    
    author: User = Relationship(back_populates="posts")
Selective Fields (Indexes, Defaults)
Pythonemail: str = Field(index=True, unique=True, max_length=255)
is_active: bool = Field(default=True)
created_at: datetime = Field(default_factory=datetime.utcnow)