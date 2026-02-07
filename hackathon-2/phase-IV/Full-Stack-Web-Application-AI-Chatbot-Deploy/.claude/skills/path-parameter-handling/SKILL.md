---
name: path-parameter-handling
description: Safely handles path parameters with validation, type conversion, existence checks, and proper error responses for invalid or missing resources.
---

# Path Parameter Handling Skill

Clean path param handling prevents bugs and improves API reliability.

## Best Practices
- Use typed parameters: int, UUID, str
- Validate format early
- Check existence before processing
- Return 404 for not found, 400 for invalid format
- Use dependencies for common checks

## Standard Pattern

```python
from fastapi import Path, HTTPException

@router.get("/users/{user_id}")
async def get_user(
    user_id: int = Path(..., ge=1, description="User ID must be positive integer")
):
    user = await crud.get_user(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user
UUID Handling
Pythonfrom uuid import UUID

@router.get("/posts/{post_id}")
async def get_post(post_id: UUID):