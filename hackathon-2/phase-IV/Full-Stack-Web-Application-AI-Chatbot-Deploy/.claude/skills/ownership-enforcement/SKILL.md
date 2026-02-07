---
name: ownership-enforcement
description: Strictly enforces resource ownership on update/delete operations by verifying current user owns the resource before allowing modifications.
---

# Ownership Enforcement Skill

Prevents users from modifying others' data.

## Standard Ownership Check

```python
async def get_post_or_404(
    post_id: int,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    post = await crud.get_post(db, post_id)
    if not post:
        raise HTTPException(404, "Post not found")
    
    if post.author_id != current_user.id and not current_user.is_admin:
        raise HTTPException(403, "Not authorized to access this post")
    
    return post
Usage in Endpoints
Python@router.delete("/posts/{post_id}")
async def delete_post(post: Post = Depends(get_post_or_404)):
    await crud.delete_post(db, post)
    return {"message": "Deleted"}