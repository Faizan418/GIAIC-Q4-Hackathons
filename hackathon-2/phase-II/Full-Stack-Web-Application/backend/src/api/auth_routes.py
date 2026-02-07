from fastapi import APIRouter, Depends, HTTPException, status
from sqlmodel.ext.asyncio.session import AsyncSession
from datetime import timedelta
from ..auth import (
    UserLogin, UserRegister, UserResponse, Token,
    authenticate_user, create_access_token,
    ACCESS_TOKEN_EXPIRE_MINUTES, get_password_hash,
    get_current_user_from_token
)
from ..database.db import get_async_session
from ..models.user import User
from sqlmodel import select
import uuid

router = APIRouter(prefix="/auth", tags=["authentication"])

@router.post("/register", response_model=UserResponse)
async def register_user(user_data: UserRegister, session: AsyncSession = Depends(get_async_session)):
    # Check if user already exists
    existing_user_statement = select(User).where(User.email == user_data.email)
    result = await session.exec(existing_user_statement)
    existing_user = result.first()

    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="User with this email already exists"
        )

    # Create new user
    user_id = str(uuid.uuid4())
    hashed_password = get_password_hash(user_data.password)
    db_user = User(
        id=user_id,
        email=user_data.email,
        name=user_data.name,
        password_hash=hashed_password
    )

    session.add(db_user)
    await session.commit()
    await session.refresh(db_user)

    return UserResponse(id=db_user.id, email=db_user.email, name=db_user.name)


@router.post("/login", response_model=Token)
async def login_user(user_credentials: UserLogin, session: AsyncSession = Depends(get_async_session)):
    user = await authenticate_user(user_credentials.email, user_credentials.password, session)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.email, "user_id": user.id}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}


@router.get("/me", response_model=UserResponse)
async def read_users_me(current_user: User = Depends(get_current_user_from_token)):
    return UserResponse(id=current_user.id, email=current_user.email, name=getattr(current_user, 'name', None))