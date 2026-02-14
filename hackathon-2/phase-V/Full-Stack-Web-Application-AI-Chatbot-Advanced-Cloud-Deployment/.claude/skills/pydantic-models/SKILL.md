name: pydantic-models
description: Defines clean Pydantic v2 models for request validation, response serialization, and separation from SQLModel table definitions.
---

# Pydantic Models Skill

Pydantic v2 (2026 standard) provides faster validation and better type safety.

## Best Practice: Separate SQLModel (DB) from Pydantic (API)

```python
# schemas/user.py
class UserCreate(BaseModel):
    email: EmailStr
    password: str = Field(min_length=12)
    name: str

class UserRead(BaseModel):
    id: int
    email: EmailStr
    name: str
    is_active: bool

    model_config = ConfigDict(from_attributes=True)

class UserUpdate(BaseModel):
    name: Optional[str] = None
    password: Optional[str] = None
Usage in Routes
Python@router.post("/users", response_model=UserRead)
async def create_user(user_in: UserCreate, db: Session = Depends(get_db)):
    db_user = User.from_orm(user_in)  # or manual mapping
    db.add(db_user)
    db.commit()
    return db_user
Model Config Tips

Use ConfigDict(from_attributes=True) for ORM mode
Exclude secrets (hashed_password)
Nested models for complex responses