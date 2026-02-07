---
name: fastapi-routing
description: Organizes FastAPI routes using APIRouter for modular, versioned, tagged endpoints with proper prefixes, dependencies, and response models.
---

# FastAPI Routing Skill

This skill creates clean, scalable API routing structure using **APIRouter** for modularity.

## Recommended Project Structure
backend/
├── app/
│   ├── main.py                # FastAPI app instance + include routers
│   ├── api/
│   │   ├── v1/
│   │   │   ├── init.py
│   │   │   ├── router.py      # Main v1 router
│   │   │   ├── auth_router.py # Auth routes
│   │   │   ├── users_router.py
│   │   │   └── items_router.py
│   ├── core/                  # Config, security
│   ├── models/                # SQLModel models
│   ├── schemas/               # Pydantic request/response
│   ├── crud/                  # Database operations
│   └── dependencies/          # Reusable Depends
└── requirements.txt
text## Best Practices
- Use APIRouter per domain/feature
- Prefix with version (/api/v1)
- Add tags for OpenAPI grouping
- Apply common dependencies/responses at router level
- Include routers in main.py

## Example Router Setup

```python
# app/api/v1/users_router.py
from fastapi import APIRouter, Depends

router = APIRouter(prefix="/users", tags=["Users"])

@router.get("/")
async def read_users():
    return {"message": "All users"}

@router.get("/{user_id}")
async def read_user(user_id: int):
    return {"user_id": user_id}
Main App Inclusion
Python# app/main.py
from fastapi import FastAPI
from app.api.v1.router import api_router

app = FastAPI(title="My API")

app.include_router(api_router, prefix="/api/v1")