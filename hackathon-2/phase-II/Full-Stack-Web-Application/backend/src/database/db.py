from sqlmodel import SQLModel
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from typing import AsyncGenerator
import os
from dotenv import load_dotenv

# Load environment variables from root .env file
load_dotenv()

# Get database URL from environment variables
# Use Neon database if DATABASE_URL is set, otherwise fallback to SQLite for development
DATABASE_URL_ENV = os.getenv("DATABASE_URL", "").strip('"\'')  # Remove quotes if present
if DATABASE_URL_ENV and "postgresql" in DATABASE_URL_ENV:
    # Use the Neon PostgreSQL database if properly configured
    DATABASE_URL = DATABASE_URL_ENV
else:
    # Fallback to SQLite for local development without Neon
    DATABASE_URL = "sqlite:///./taskapp_dev.db"

# Ensure we're using the right driver for async operations
if "postgresql" in DATABASE_URL and "+asyncpg" not in DATABASE_URL:
    DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://")
elif "sqlite" in DATABASE_URL and "+aiosqlite" not in DATABASE_URL:
    # For SQLite, we'll use aiosqlite for async operations
    DATABASE_URL = DATABASE_URL.replace("sqlite://", "sqlite+aiosqlite://")

# Handle asyncpg-specific URL parameters - remove unsupported parameters
if "postgresql+asyncpg://" in DATABASE_URL and "?" in DATABASE_URL:
    base_url, query_params = DATABASE_URL.split('?', 1)
    # Parse query parameters and remove unsupported ones for asyncpg
    params_list = query_params.split('&')
    filtered_params = [param for param in params_list
                      if not param.startswith('sslmode=') and not param.startswith('channel_binding=')]
    if filtered_params:
        DATABASE_URL = f"{base_url}?{'&'.join(filtered_params)}"
    else:
        DATABASE_URL = base_url

# Create async engine with connection settings optimized for Neon serverless PostgreSQL
async_engine = create_async_engine(
    DATABASE_URL,
    # Connection settings optimized for Neon serverless PostgreSQL
    pool_size=2,  # Smaller pool for serverless
    max_overflow=5,  # Limited overflow
    pool_pre_ping=True,  # Verify connections before use (important for serverless)
    pool_recycle=300,  # Recycle connections to prevent serverless timeout issues
    echo=False  # Set to True for SQL query logging
)

# Create async session maker
AsyncSessionLocal = sessionmaker(
    async_engine,
    class_=AsyncSession,
    expire_on_commit=False
)

async def get_async_session() -> AsyncGenerator[AsyncSession, None]:
    """Get an async database session"""
    async with AsyncSessionLocal() as session:
        yield session


# Import models to register them with SQLModel metadata
from ..models.user import User
from ..models.task import Task

async def create_db_and_tables():
    """Create database tables"""
    async with async_engine.begin() as conn:
        # Use run_sync to execute the sync method create_all within an async context
        await conn.run_sync(SQLModel.metadata.create_all)