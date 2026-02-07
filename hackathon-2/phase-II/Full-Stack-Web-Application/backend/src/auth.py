from datetime import datetime, timedelta
from typing import Optional
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import jwt
import os
from pydantic import BaseModel
from sqlmodel.ext.asyncio.session import AsyncSession
from passlib.context import CryptContext
from .database.db import get_async_session
from .models.user import User

# Security context for password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT settings
SECRET_KEY = os.getenv("BETTER_AUTH_SECRET") or os.getenv("JWT_SECRET_KEY") or "fallback-test-secret"
ALGORITHM = os.getenv("JWT_ALGORITHM", "HS256")
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("JWT_ACCESS_TOKEN_EXPIRE_MINUTES", 30))

# Security scheme for API docs
security = HTTPBearer()

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    username: Optional[str] = None

class UserLogin(BaseModel):
    email: str
    password: str

class UserRegister(UserLogin):
    name: str

class UserResponse(BaseModel):
    id: str
    email: str
    name: Optional[str] = None

def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

async def authenticate_user(email: str, password: str, session: AsyncSession) -> Optional[User]:
    # Find user by email field
    from sqlmodel import select
    statement = select(User).where(User.email == email)
    result = await session.exec(statement)
    user = result.first()

    if not user or not verify_password(password, user.password_hash):
        return None
    return user

async def get_current_user_from_token(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    session: AsyncSession = Depends(get_async_session)
) -> User:
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(credentials.credentials, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            raise credentials_exception
        token_data = TokenData(username=email)
    except jwt.PyJWTError:
        raise credentials_exception

    from sqlmodel import select
    statement = select(User).where(User.email == token_data.username)
    result = await session.exec(statement)
    user = result.first()

    if user is None:
        raise credentials_exception
    return user