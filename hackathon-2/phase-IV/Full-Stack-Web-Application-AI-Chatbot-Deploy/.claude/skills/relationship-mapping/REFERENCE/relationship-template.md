## Cascade Delete
```python
posts: List["Post"] = Relationship(
    back_populates="author",
    cascade_delete=True
)
Lazy vs Eager Loading
Python# Default: lazy (selectin)
# For frequent access: selectinload
from sqlmodel import select
from sqlalchemy.orm import selectinload

stmt = select(User).options(selectinload(User.posts))