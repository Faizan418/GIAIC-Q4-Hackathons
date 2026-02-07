"""Authentication API endpoints for the RAG Chatbot API."""

import logging
from datetime import timedelta

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm

from backend.config import get_settings
from backend.models.auth import (
    AuthResponse,
    Token,
    UserCreate,
    UserProfileCreate,
    UserProfilePublic,
    UserProfileUpdate,
    UserPublic,
)
from backend.services.auth_service import (
    authenticate_user,
    create_access_token,
    create_user,
    create_user_profile,
    get_or_create_user_profile,
    is_profile_complete,
    update_user_profile,
)

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/auth", tags=["auth"])

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="auth/token")

# Get settings for token configuration
settings = get_settings()


@router.post("/register", response_model=AuthResponse)
async def register(user: UserCreate):
    """Register a new user."""
    try:
        # Create the user
        user_in_db = await create_user(user)

        # Create access token
        access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
        access_token = create_access_token(
            data={"user_id": user_in_db.id, "email": user_in_db.email},
            expires_delta=access_token_expires,
        )

        # Create public user response
        user_public = UserPublic(
            id=user_in_db.id,
            email=user_in_db.email,
            created_at=user_in_db.created_at,
            updated_at=user_in_db.updated_at,
            email_verified=user_in_db.email_verified,
            profile_complete=False,  # Profile is not complete initially
        )

        return AuthResponse(
            user=user_public, access_token=access_token, token_type="bearer"
        )
    except ValueError as e:
        # Email already exists
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(e))
    except Exception as e:
        logger.error(f"Registration error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during registration",
        )


@router.post("/token", response_model=Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    """Login endpoint that returns an access token."""
    user = await authenticate_user(form_data.username, form_data.password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
    access_token = create_access_token(
        data={"user_id": user.id, "email": user.email},
        expires_delta=access_token_expires,
    )

    return Token(access_token=access_token, token_type="bearer")


@router.get("/profile", response_model=UserProfilePublic)
async def get_profile(token: str = Depends(oauth2_scheme)):
    """Get user profile."""
    from jose import jwt

    from backend.models.auth import TokenData
    from backend.services.auth_service import get_user_by_id

    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    try:
        payload = jwt.decode(
            token, settings.secret_key, algorithms=[settings.algorithm]
        )
        user_id: str = payload.get("user_id")
        if user_id is None:
            raise credentials_exception
        token_data = TokenData(user_id=user_id)
    except jwt.ExpiredSignatureError:
        raise credentials_exception
    except jwt.JWTError:
        raise credentials_exception

    user = await get_user_by_id(token_data.user_id)
    if user is None:
        raise credentials_exception

    profile = await get_or_create_user_profile(token_data.user_id)
    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND, detail="User profile not found"
        )

    return profile


@router.post("/profile", response_model=UserProfilePublic)
async def create_profile(
    profile: UserProfileCreate, token: str = Depends(oauth2_scheme)
):
    """Create or update user profile."""
    from jose import jwt

    from backend.models.auth import TokenData
    from backend.services.auth_service import get_user_by_id

    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    try:
        payload = jwt.decode(
            token, settings.secret_key, algorithms=[settings.algorithm]
        )
        user_id: str = payload.get("user_id")
        if user_id is None:
            raise credentials_exception
        token_data = TokenData(user_id=user_id)
    except jwt.ExpiredSignatureError:
        raise credentials_exception
    except jwt.JWTError:
        raise credentials_exception

    # Verify the user exists
    user = await get_user_by_id(token_data.user_id)
    if user is None:
        raise credentials_exception

    # Ensure the profile is associated with the authenticated user
    profile.user_id = token_data.user_id

    # Create or update the profile
    created_profile = await create_user_profile(profile)
    if not created_profile:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create profile",
        )

    return created_profile


@router.patch("/profile", response_model=UserProfilePublic)
async def update_profile(
    profile_update: UserProfileUpdate, token: str = Depends(oauth2_scheme)
):
    """Update user profile."""
    from jose import jwt

    from backend.models.auth import TokenData
    from backend.services.auth_service import get_user_by_id

    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    try:
        payload = jwt.decode(
            token, settings.secret_key, algorithms=[settings.algorithm]
        )
        user_id: str = payload.get("user_id")
        if user_id is None:
            raise credentials_exception
        token_data = TokenData(user_id=user_id)
    except jwt.ExpiredSignatureError:
        raise credentials_exception
    except jwt.JWTError:
        raise credentials_exception

    # Verify the user exists
    user = await get_user_by_id(token_data.user_id)
    if user is None:
        raise credentials_exception

    # Convert the Pydantic model to a dict to pass to the service
    profile_data = profile_update.dict(exclude_unset=True)

    updated_profile = await update_user_profile(token_data.user_id, profile_data)
    if not updated_profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND, detail="User profile not found"
        )

    return updated_profile


@router.get("/me", response_model=UserPublic)
async def get_current_user(token: str = Depends(oauth2_scheme)):
    """Get current authenticated user."""
    from jose import jwt

    from backend.models.auth import TokenData
    from backend.services.auth_service import get_user_by_id

    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    try:
        payload = jwt.decode(
            token, settings.secret_key, algorithms=[settings.algorithm]
        )
        user_id: str = payload.get("user_id")
        email: str = payload.get("email")
        if user_id is None:
            raise credentials_exception
        token_data = TokenData(user_id=user_id, email=email)
    except jwt.ExpiredSignatureError:
        raise credentials_exception
    except jwt.JWTError:
        raise credentials_exception

    user = await get_user_by_id(token_data.user_id)
    if user is None:
        raise credentials_exception

    # Check if profile is complete
    profile_complete = await is_profile_complete(token_data.user_id)

    return UserPublic(
        id=user.id,
        email=user.email,
        created_at=user.created_at,
        updated_at=user.updated_at,
        email_verified=user.email_verified,
        profile_complete=profile_complete,
    )


@router.get("/profile-complete", response_model=dict)
async def check_profile_complete(token: str = Depends(oauth2_scheme)):
    """Check if user profile is complete."""
    from jose import jwt

    from backend.models.auth import TokenData
    from backend.services.auth_service import get_user_by_id

    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    try:
        payload = jwt.decode(
            token, settings.secret_key, algorithms=[settings.algorithm]
        )
        user_id: str = payload.get("user_id")
        if user_id is None:
            raise credentials_exception
        token_data = TokenData(user_id=user_id)
    except jwt.ExpiredSignatureError:
        raise credentials_exception
    except jwt.JWTError:
        raise credentials_exception

    user = await get_user_by_id(token_data.user_id)
    if user is None:
        raise credentials_exception

    profile_complete = await is_profile_complete(token_data.user_id)

    return {"profile_complete": profile_complete}
