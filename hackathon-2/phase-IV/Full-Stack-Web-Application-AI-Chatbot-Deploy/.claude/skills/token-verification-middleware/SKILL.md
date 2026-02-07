---
name: token-verification-middleware
description: Implements reusable token verification middleware using JWT, HttpOnly cookies, or headers with proper error handling and user object injection.
---

# Token Verification Middleware Skill

Centralized token verification prevents duplication and ensures consistency.

## Recommended Approach
- Extract token from Authorization header or cookie
- Verify signature and expiry
- Inject current_user into endpoint
- Handle invalid/expired gracefully

## Reusable Dependency

```python
from fastapi import Depends, HTTPException, Request

async def get_current_user(request: Request):
    token = request.cookies.get("access_token") or request.headers.get("Authorization", "").removeprefix("Bearer ").strip()
    if not token:
        raise HTTPException(401, detail="Missing token")
    
    payload = verify_jwt(token)
    if not payload:
        raise HTTPException(401, detail="Invalid or expired token")
    
    user = await get_user_by_id(payload.sub)
    if not user or not user.is_active:
        raise HTTPException(401, detail="Inactive user")
    
    return user
Usage
Python@router.get("/me")
async def read_me(current_user: User = Depends(get_current_user)):
    return current_user