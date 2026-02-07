"""Authentication models for the RAG Chatbot API."""

from datetime import datetime
from enum import Enum
from typing import List, Optional

from pydantic import BaseModel, Field


class SoftwareLevel(str, Enum):
    BEGINNER = "BEGINNER"
    INTERMEDIATE = "INTERMEDIATE"
    ADVANCED = "ADVANCED"


class HardwareExperience(str, Enum):
    NONE = "NONE"
    BASIC = "BASIC"
    INTERMEDIATE = "INTERMEDIATE"
    ADVANCED = "ADVANCED"


class LearningTrack(str, Enum):
    SOFTWARE_ONLY = "SOFTWARE_ONLY"
    HARDWARE_ONLY = "HARDWARE_ONLY"
    FULL_ROBOTICS = "FULL_ROBOTICS"


class KnownLanguage(str, Enum):
    PYTHON = "Python"
    JAVASCRIPT = "JavaScript"
    CPP = "C++"
    OTHER = "Other"


class BoardUsed(str, Enum):
    ARDUINO = "Arduino"
    ESP32 = "ESP32"
    RASPBERRY_PI = "Raspberry Pi"


class UserBase(BaseModel):
    """Base model for user data."""

    email: str = Field(..., description="User's email address")


class UserCreate(UserBase):
    """Model for creating a new user."""

    password: str = Field(..., min_length=8, description="User's password")
    email: str = Field(..., description="User's email address")


class UserUpdate(BaseModel):
    """Model for updating user information."""

    email: Optional[str] = None


class UserInDB(UserBase):
    """Model for user data as stored in the database."""

    id: str
    email: str
    hashed_password: str
    created_at: datetime
    updated_at: datetime
    email_verified: bool = False

    class Config:
        from_attributes = True


class UserProfileBase(BaseModel):
    """Base model for user profile data."""

    software_level: SoftwareLevel
    known_languages: List[KnownLanguage]
    hardware_experience: HardwareExperience
    learning_track: LearningTrack
    boards_used: Optional[List[BoardUsed]] = []


class UserProfileCreate(UserProfileBase):
    """Model for creating a user profile."""

    user_id: str


class UserProfileUpdate(BaseModel):
    """Model for updating user profile."""

    software_level: Optional[SoftwareLevel] = None
    known_languages: Optional[List[KnownLanguage]] = None
    hardware_experience: Optional[HardwareExperience] = None
    learning_track: Optional[LearningTrack] = None
    boards_used: Optional[List[BoardUsed]] = None


class UserProfileInDB(UserProfileBase):
    """Model for user profile data as stored in the database."""

    id: str
    user_id: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class UserPublic(UserBase):
    """Public model for user data (without sensitive information)."""

    id: str
    created_at: datetime
    updated_at: datetime
    email_verified: bool = False
    profile_complete: bool = False

    class Config:
        from_attributes = True


class UserProfilePublic(UserProfileBase):
    """Public model for user profile data."""

    id: str
    user_id: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class Token(BaseModel):
    """Model for authentication token."""

    access_token: str
    token_type: str


class TokenData(BaseModel):
    """Model for token data."""

    user_id: Optional[str] = None
    email: Optional[str] = None


class AuthResponse(BaseModel):
    """Model for authentication response."""

    user: UserPublic
    access_token: str
    token_type: str = "bearer"
