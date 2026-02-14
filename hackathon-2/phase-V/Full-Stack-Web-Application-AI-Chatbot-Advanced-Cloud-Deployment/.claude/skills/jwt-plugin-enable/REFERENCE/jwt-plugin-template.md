# JWT Plugin Configuration

## fastapi-users Full Setup
```python
from fastapi_users.db import SQLAlchemyUserDatabase

async def get_user_db():
    yield SQLAlchemyUserDatabase(User, session)

fastapi_users = FastAPIUsers(
    get_user_manager,
    [auth_backend],
)

# Protected route
@router.get("/protected")
async def protected_route(user: User = Depends(fastapi_users.current_user(active=True))):
    return {"message": f"Hello {user.email}"}
Custom Claims
Pythondef get_jwt_strategy():
    return JWTStrategy(
        secret=SECRET,
        lifetime_seconds=3600,
        token_audience=["myapp:auth"],
        # Custom claims
        # algorithm="HS256"
    )