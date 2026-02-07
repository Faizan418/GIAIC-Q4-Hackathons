# Reusable Dependency Examples

## Rate Limiting
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

def rate_limit():
    return limiter.shared_limit("100/hour", scope="user")

@router.get("/limited")
@limiter.limit("5/minute")
async def limited_endpoint():
    return {"message": "OK"}
Permission Check
Pythondef require_role(role: str):
    def role_checker(current_user: User = Depends(get_current_user)):
        if current_user.role != role:
            raise PermissionError()
        return current_user
    return role_checker

@router.post("/admin")
async def admin_only(admin: User = Depends(require_role("admin"))):
    ...