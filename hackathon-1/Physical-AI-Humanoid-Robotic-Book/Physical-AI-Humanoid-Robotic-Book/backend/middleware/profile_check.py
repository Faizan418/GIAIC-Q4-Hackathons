"""Profile completeness middleware/check for the RAG Chatbot API."""

from fastapi import HTTPException, Request, status
from fastapi.security import HTTPBearer
from jose import jwt

from backend.config import get_settings
from backend.services.auth_service import get_or_create_user_profile

security = HTTPBearer(auto_error=False)
settings = get_settings()


async def check_profile_completeness_dependency(request: Request):
    """
    Dependency to check if user profile is complete.
    This can be used as a middleware-like function for routes that require complete profiles.
    """
    # Extract token from authorization header
    auth_header = request.headers.get("authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        # No token provided, user is not authenticated
        return {"profile_complete": False, "user_authenticated": False}

    token = auth_header.split(" ")[1]

    try:
        # Decode the JWT token
        payload = jwt.decode(
            token, settings.secret_key, algorithms=[settings.algorithm]
        )
        user_id: str = payload.get("user_id")

        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Check if the user has a complete profile
        profile = await get_or_create_user_profile(user_id)
        profile_complete = profile is not None

        return {
            "profile_complete": profile_complete,
            "user_authenticated": True,
            "user_id": user_id,
        }

    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired",
            headers={"WWW-Authenticate": "Bearer"},
        )
    except jwt.JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )


async def require_profile_completeness(request: Request):
    """
    Dependency that requires a user to have a complete profile.
    Raises an HTTPException if the profile is not complete.
    """
    profile_check = await check_profile_completeness_dependency(request)

    if not profile_check["user_authenticated"]:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if not profile_check["profile_complete"]:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Profile completion required. Please complete your profile first.",
        )

    return profile_check
