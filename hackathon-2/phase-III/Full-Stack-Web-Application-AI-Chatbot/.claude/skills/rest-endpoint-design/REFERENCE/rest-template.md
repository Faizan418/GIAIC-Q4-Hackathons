# REST Endpoint Examples

## Users Resource
```python
@router.get("/users", tags=["Users"], response_model=List[UserRead])
def list_users(...):
    ...

@router.post("/users", status_code=201, response_model=UserRead)
def create_user(...):
    ...

@router.patch("/users/{user_id}", response_model=UserRead)
def partial_update_user(user_id: int, ...):
    ...
Nested Resources
Python@router.get("/users/{user_id}/posts")
def get_user_posts(user_id: int):
    ...

@router.post("/posts/{post_id}/comments")
def create_comment(post_id: int, ...):
    ...
Bulk Operations
Python@router.post("/users/bulk-delete", status_code=204)
def bulk_delete_users(ids: List[int]):
```