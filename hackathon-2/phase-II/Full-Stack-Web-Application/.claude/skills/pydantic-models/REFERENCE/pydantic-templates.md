# Pydantic v2 Model Examples

## Token Response
```python
class Token(BaseModel):
    access_token: str
    token_type: str = "bearer"
    expires_in: int = 3600

class TokenData(BaseModel):
    sub: str
    exp: Optional[int] = None
Pagination
Pythonclass PaginatedResponse[T](BaseModel):
    items: List[T]
    total: int
    page: int
    size: int

# Usage: response_model=PaginatedResponse[UserRead]
Validation with Validators
Pythonfrom pydantic import field_validator

class UserCreate(BaseModel):
    password: str

    @field_validator("password")
    @classmethod
    def validate_password(cls, v: str):
        if len(v) < 12:
            raise ValueError("Password must be at least 12 characters")
        return v