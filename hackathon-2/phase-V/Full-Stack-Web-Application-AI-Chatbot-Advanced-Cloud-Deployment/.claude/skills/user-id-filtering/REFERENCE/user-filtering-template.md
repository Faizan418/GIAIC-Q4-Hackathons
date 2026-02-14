## Posts List with Admin Bypass
```python
filters = {}
if not current_user.is_admin:
    filters["author_id"] = current_user.id

posts = await crud.get_posts(db, **filters)
Orders Endpoint
Python@router.get("/orders")
async def my_orders(current_user: User = Depends(get_current_user)):
    return await crud.get_orders_by_user(db, current_user.id)