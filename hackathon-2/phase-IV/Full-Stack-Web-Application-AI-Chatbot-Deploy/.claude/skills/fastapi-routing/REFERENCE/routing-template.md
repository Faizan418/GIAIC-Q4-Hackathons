# Modular Router Template

## Main API Router
```python
# app/api/v1/router.py
from fastapi import APIRouter
from .auth_router import router as auth_router
from .users_router import router as users_router

api_router = APIRouter()

api_router.include_router(auth_router)
api_router.include_router(users_router)
Common Dependencies at Router Level
Pythonrouter = APIRouter(
    prefix="/items",
    tags=["Items"],
    dependencies=[Depends(get_current_user)],
    responses={404: {"description": "Not found"}}
)
```