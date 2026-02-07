---
name: jwt-plugin-enable
description: Enables and configures JWT authentication plugin (like fastapi-jwt-auth or Authlib) with proper secret management, token expiry, refresh routes, and revocation support.
---

# JWT Plugin Enable Skill

Use established JWT libraries for production security.

## Recommended Libraries (2026)
- fastapi-users (most popular)
- fastapi-jwt-auth
- Authlib

## fastapi-users Example Setup
```python
from fastapi_users import FastAPIUsers
from fastapi_users.authentication import JWTStrategy, AuthenticationBackend

SECRET = os.getenv("AUTH_SECRET")

def get_jwt_strategy() -> JWTStrategy:
    return JWTStrategy(secret=SECRET, lifetime_seconds=3600)  # 1 hour

auth_backend = AuthenticationBackend(
    name="jwt",
    transport=bearer_transport,
    get_strategy=get_jwt_strategy,
)

fastapi_users = FastAPIUsers[User, uuid.UUID](
    get_user_manager,
    [auth_backend],
)

app.include_router(
    fastapi_users.get_auth_router(auth_backend),
    prefix="/auth/jwt",
    tags=["auth"],
)
Routes Provided

/auth/jwt/login
/auth/jwt/logout
/auth/jwt/refresh