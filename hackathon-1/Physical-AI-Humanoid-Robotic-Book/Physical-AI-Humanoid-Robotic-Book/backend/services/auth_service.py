"""Authentication service for the RAG Chatbot API."""

import logging
from datetime import datetime, timedelta
from typing import Optional

import asyncpg
from jose import jwt
from passlib.context import CryptContext

from backend.config import get_settings
from backend.db.neon_postgres import db
from backend.models.auth import UserCreate, UserInDB, UserProfileCreate, UserProfileInDB
from backend.utils.auth_logging import AuthLogEvent, log_auth_event

logger = logging.getLogger(__name__)

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# Get settings for JWT configuration
settings = get_settings()


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against a hashed password."""
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """Hash a password."""
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create an access token."""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=30)  # Default 30 minutes
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(
        to_encode, settings.secret_key, algorithm=settings.algorithm
    )
    return encoded_jwt


async def authenticate_user(email: str, password: str) -> Optional[UserInDB]:
    """Authenticate a user by email and password."""
    log_auth_event(
        AuthLogEvent.USER_LOGIN_START, email=email, details={"ip_address": "unknown"}
    )

    user = await get_user_by_email(email)
    if not user or not verify_password(password, user.hashed_password):
        log_auth_event(
            AuthLogEvent.USER_LOGIN_FAILURE,
            email=email,
            details={"error": "Invalid credentials"},
        )
        return None

    log_auth_event(
        AuthLogEvent.USER_LOGIN_SUCCESS,
        user_id=user.id,
        email=user.email,
        details={"user_id": user.id},
    )
    return user


async def get_user_by_email(email: str) -> Optional[UserInDB]:
    """Get a user by email."""
    if not db.pool:
        raise Exception("Database not connected")

    async with db.pool.acquire() as conn:
        # Create users table if it doesn't exist
        await conn.execute(
            """
            CREATE TABLE IF NOT EXISTS users (
                id VARCHAR(255) PRIMARY KEY,
                email VARCHAR(255) UNIQUE NOT NULL,
                hashed_password VARCHAR(255) NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                email_verified BOOLEAN DEFAULT FALSE
            )
        """
        )

        row = await conn.fetchrow(
            "SELECT id, email, hashed_password, created_at, updated_at, email_verified FROM users WHERE email = $1",
            email,
        )

        if row:
            pass

            # Convert asyncpg record to dict
            user_data = {
                "id": row["id"],
                "email": row["email"],
                "hashed_password": row["hashed_password"],
                "created_at": row["created_at"],
                "updated_at": row["updated_at"],
                "email_verified": row["email_verified"],
            }

            # Create UserInDB instance manually to avoid parsing issues
            return UserInDB(
                id=user_data["id"],
                email=user_data["email"],
                hashed_password=user_data["hashed_password"],
                created_at=user_data["created_at"],
                updated_at=user_data["updated_at"],
                email_verified=user_data["email_verified"],
            )
        return None


async def get_user_by_id(user_id: str) -> Optional[UserInDB]:
    """Get a user by ID."""
    if not db.pool:
        raise Exception("Database not connected")

    async with db.pool.acquire() as conn:
        row = await conn.fetchrow(
            "SELECT id, email, hashed_password, created_at, updated_at, email_verified FROM users WHERE id = $1",
            user_id,
        )

        if row:
            return UserInDB(
                id=row["id"],
                email=row["email"],
                hashed_password=row["hashed_password"],
                created_at=row["created_at"],
                updated_at=row["updated_at"],
                email_verified=row["email_verified"],
            )
        return None


async def create_user(user: UserCreate) -> UserInDB:
    """Create a new user."""
    log_auth_event(
        AuthLogEvent.USER_REGISTRATION_START,
        email=user.email,
        details={"ip_address": "unknown"},
    )

    if not db.pool:
        raise Exception("Database not connected")

    async with db.pool.acquire() as conn:
        # Create users table if it doesn't exist
        await conn.execute(
            """
            CREATE TABLE IF NOT EXISTS users (
                id VARCHAR(255) PRIMARY KEY,
                email VARCHAR(255) UNIQUE NOT NULL,
                hashed_password VARCHAR(255) NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                email_verified BOOLEAN DEFAULT FALSE
            )
        """
        )

        # Generate a unique ID for the user
        import uuid

        user_id = str(uuid.uuid4())

        # Hash the password
        hashed_password = get_password_hash(user.password)

        try:
            # Insert the new user
            user_id = await conn.fetchval(
                """
                INSERT INTO users (id, email, hashed_password, created_at, updated_at)
                VALUES ($1, $2, $3, $4, $5)
                RETURNING id
                """,
                user_id,
                user.email,
                hashed_password,
                datetime.utcnow(),
                datetime.utcnow(),
            )

            # Return the created user
            created_user = await get_user_by_id(user_id)
            log_auth_event(
                AuthLogEvent.USER_REGISTRATION_SUCCESS,
                user_id=created_user.id,
                email=created_user.email,
                details={"user_id": created_user.id},
            )
            return created_user
        except asyncpg.UniqueViolationError:
            # Email already exists
            log_auth_event(
                AuthLogEvent.USER_REGISTRATION_FAILURE,
                email=user.email,
                details={"error": "Email already registered"},
            )
            raise ValueError("Email already registered")


async def get_or_create_user_profile(user_id: str) -> Optional[UserProfileInDB]:
    """Get a user profile or create a default one if it doesn't exist."""
    if not db.pool:
        raise Exception("Database not connected")

    async with db.pool.acquire() as conn:
        # Create user_profiles table if it doesn't exist
        await conn.execute(
            """
            CREATE TABLE IF NOT EXISTS user_profiles (
                id VARCHAR(255) PRIMARY KEY,
                user_id VARCHAR(255) UNIQUE NOT NULL,
                software_level VARCHAR(20) NOT NULL,
                known_languages TEXT[] DEFAULT '{}',
                hardware_experience VARCHAR(20) NOT NULL,
                learning_track VARCHAR(20) NOT NULL,
                boards_used TEXT[] DEFAULT '{}',
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
            )
        """
        )

        # Try to get existing profile
        row = await conn.fetchrow(
            """
            SELECT id, user_id, software_level, known_languages, hardware_experience,
                   learning_track, boards_used, created_at, updated_at
            FROM user_profiles WHERE user_id = $1
            """,
            user_id,
        )

        if row:
            # Convert known_languages and boards_used from PostgreSQL arrays to Python lists
            known_languages = row["known_languages"] if row["known_languages"] else []
            boards_used = row["boards_used"] if row["boards_used"] else []

            return UserProfileInDB(
                id=row["id"],
                user_id=row["user_id"],
                software_level=row["software_level"],
                known_languages=known_languages,
                hardware_experience=row["hardware_experience"],
                learning_track=row["learning_track"],
                boards_used=boards_used,
                created_at=row["created_at"],
                updated_at=row["updated_at"],
            )

        # Profile doesn't exist, return None
        return None


async def create_user_profile(profile: UserProfileCreate) -> UserProfileInDB:
    """Create a user profile."""
    if not db.pool:
        raise Exception("Database not connected")

    async with db.pool.acquire() as conn:
        # Create user_profiles table if it doesn't exist (same as above)
        await conn.execute(
            """
            CREATE TABLE IF NOT EXISTS user_profiles (
                id VARCHAR(255) PRIMARY KEY,
                user_id VARCHAR(255) UNIQUE NOT NULL,
                software_level VARCHAR(20) NOT NULL,
                known_languages TEXT[] DEFAULT '{}',
                hardware_experience VARCHAR(20) NOT NULL,
                learning_track VARCHAR(20) NOT NULL,
                boards_used TEXT[] DEFAULT '{}',
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
            )
        """
        )

        import uuid

        profile_id = str(uuid.uuid4())

        # Convert lists to PostgreSQL arrays
        known_languages_array = (
            "{" + ",".join([f'"{lang}"' for lang in profile.known_languages]) + "}"
        )
        boards_used_array = (
            "{" + ",".join([f'"{board}"' for board in profile.boards_used or []]) + "}"
            if profile.boards_used
            else "{}"
        )

        try:
            profile_id = await conn.fetchval(
                """
                INSERT INTO user_profiles (id, user_id, software_level, known_languages,
                                         hardware_experience, learning_track, boards_used, created_at, updated_at)
                VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
                ON CONFLICT (user_id) DO UPDATE SET
                    software_level = EXCLUDED.software_level,
                    known_languages = EXCLUDED.known_languages,
                    hardware_experience = EXCLUDED.hardware_experience,
                    learning_track = EXCLUDED.learning_track,
                    boards_used = EXCLUDED.boards_used,
                    updated_at = EXCLUDED.updated_at
                RETURNING id
                """,
                profile_id,
                profile.user_id,
                profile.software_level.value
                if hasattr(profile.software_level, "value")
                else profile.software_level,
                known_languages_array,
                (
                    profile.hardware_experience.value
                    if hasattr(profile.hardware_experience, "value")
                    else profile.hardware_experience
                ),
                profile.learning_track.value
                if hasattr(profile.learning_track, "value")
                else profile.learning_track,
                boards_used_array,
                datetime.utcnow(),
                datetime.utcnow(),
            )

            # Return the created/updated profile
            return await get_or_create_user_profile(profile.user_id)
        except Exception as e:
            logger.error(f"Error creating user profile: {e}")
            raise


async def update_user_profile(
    user_id: str, profile_update_data: dict
) -> Optional[UserProfileInDB]:
    """Update a user profile."""
    if not db.pool:
        raise Exception("Database not connected")

    # Prepare update fields and values
    update_fields = []
    values = []
    param_index = 1

    for field, value in profile_update_data.items():
        if value is not None:
            if field in ["software_level", "hardware_experience", "learning_track"]:
                # Handle enum values
                update_fields.append(f"{field} = ${param_index}")
                values.append(value.value if hasattr(value, "value") else value)
                param_index += 1
            elif field in ["known_languages", "boards_used"]:
                # Handle array fields
                if isinstance(value, list):
                    array_str = "{" + ",".join([f'"{item}"' for item in value]) + "}"
                    update_fields.append(f"{field} = ${param_index}")
                    values.append(array_str)
                    param_index += 1
            else:
                # Handle regular fields
                update_fields.append(f"{field} = ${param_index}")
                values.append(value)
                param_index += 1

    if not update_fields:
        return await get_or_create_user_profile(user_id)

    async with db.pool.acquire() as conn:
        # Add user_id and updated_at to the query
        update_fields.append(f"updated_at = ${param_index}")
        values.append(datetime.utcnow())
        param_index += 1

        query = f"""
            UPDATE user_profiles
            SET {', '.join(update_fields)}
            WHERE user_id = ${param_index}
            RETURNING id, user_id, software_level, known_languages, hardware_experience,
                    learning_track, boards_used, created_at, updated_at
        """
        values.append(user_id)

        row = await conn.fetchrow(query, *values)

        if row:
            # Convert known_languages and boards_used from PostgreSQL arrays to Python lists
            known_languages = row["known_languages"] if row["known_languages"] else []
            boards_used = row["boards_used"] if row["boards_used"] else []

            return UserProfileInDB(
                id=row["id"],
                user_id=row["user_id"],
                software_level=row["software_level"],
                known_languages=known_languages,
                hardware_experience=row["hardware_experience"],
                learning_track=row["learning_track"],
                boards_used=boards_used,
                created_at=row["created_at"],
                updated_at=row["updated_at"],
            )
        return None


async def is_profile_complete(user_id: str) -> bool:
    """Check if a user's profile is complete."""
    profile = await get_or_create_user_profile(user_id)
    return profile is not None
