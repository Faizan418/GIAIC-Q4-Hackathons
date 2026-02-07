---
name: user-id-filtering
description: Automatically filters query results by current user ID for list endpoints to prevent data leakage while allowing admin overrides.
---

# User-ID Filtering Skill

Automatic filtering ensures users only see their own data.

## Pattern for List Endpoints

```python
@router.get("/posts")
async def list_posts(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    query = select(Post)
    
    if not current_user.is_admin:
        query = query.where(Post.author_id == current_user.id)
    
    posts = await db.execute(query)
    return posts.scalars().all()
Reusable Dependency
Pythondef get_user_filter(current_user: User = Depends(get_current_user)):
    return {"author_id": current_user.id} if not current_user.is_admin else {}