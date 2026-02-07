## Update Post
```python
@router.patch("/posts/{post_id}")
async def update_post(
    post_update: PostUpdate,
    post: Post = Depends(get_post_or_404)
):
    return await crud.update_post(db, post, post_update)
Admin Override
Pythonif current_user.is_admin:
    # Allow access to any resource
    post = await crud.get_post(db, post_id)
else:
    post = await crud.get_post_by_owner(db, post_id, current_user.id)
Audit Log Addition
Python# After ownership check
await audit_log(
    action="post_updated",
    user_id=current_user.id,
    resource_id=post.id
)