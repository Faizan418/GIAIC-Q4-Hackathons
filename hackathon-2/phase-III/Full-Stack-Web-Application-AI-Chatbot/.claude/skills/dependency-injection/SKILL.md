---
name: dependency-injection
description: Leverages FastAPI's powerful Depends system for reusable authentication, database sessions, rate limiting, and business logic injection.
---

# Dependency Injection Skill

FastAPI's DI is one of its superpowers – declare what you need, get it injected.

## Common Dependency Patterns

### Database Session
```python
from sqlmodel import Session

def get_db():
    with Session(engine) as session:
        yield session

# Usage
@router.get("/items")
async def read_items(db: Session = Depends(get_db)):
    return db.query(Item).all()
Current User
Pythonasync def get_current_user(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)):
    user = authenticate(token, db)
    if not user:
        raise AuthenticationError()
    return user

# Usage
@router.get("/me")
async def read_me(current_user: User = Depends(get_current_user)):
    return current_user