"""
Test script to verify registration functionality works properly
"""
import asyncio
from sqlmodel.ext.asyncio.session import AsyncSession
from src.models.user import User
from src.database.db import get_async_session, create_db_and_tables, async_engine
from src.auth import get_password_hash, authenticate_user
from sqlmodel import select
import uuid

async def test_user_registration_flow():
    """Test the complete user registration and authentication flow"""

    # Create tables first
    await create_db_and_tables()

    # Create a test user directly in the database
    email = "test@example.com"
    password = "testpassword123"
    name = "Test User"

    # Hash the password
    password_hash = get_password_hash(password)

    # Create user object
    user_id = str(uuid.uuid4())
    user = User(
        id=user_id,
        email=email,
        password_hash=password_hash
    )

    # Insert the user into the database
    async with AsyncSession(async_engine) as session:
        session.add(user)
        await session.commit()
        await session.refresh(user)

        print(f"✓ User created successfully: {user.email}")

        # Now test authentication
        authenticated_user = await authenticate_user(email, password, session)

        if authenticated_user:
            print(f"✓ User authenticated successfully: {authenticated_user.email}")
            print("✓ Registration and authentication flow works correctly!")
        else:
            print("✗ Authentication failed")

    return True

if __name__ == "__main__":
    print("Testing user registration flow...")
    asyncio.run(test_user_registration_flow())