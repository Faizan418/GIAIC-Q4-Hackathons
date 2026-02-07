"""
Test script to verify database connection and initialization
"""
import asyncio
from sqlmodel import SQLModel, select
from sqlmodel.ext.asyncio.session import AsyncSession
from src.database.db import async_engine, create_db_and_tables
from src.models.user import User
from src.models.task import Task
import uuid

async def test_database():
    print("Testing database connection and initialization...")

    try:
        # Try to create tables first
        print("Creating database tables...")
        await create_db_and_tables()
        print("[OK] Tables created successfully")

        # Test database connection
        print("Testing database connection...")
        async with AsyncSession(async_engine) as session:
            # Try to count users
            statement = select(User)
            result = await session.exec(statement)
            users = result.all()
            print(f"[OK] Connected to database, found {len(users)} existing users")

            # Try to create a test user
            print("Creating test user...")
            test_user = User(
                email="test_db@example.com",
                name="Test DB User",
                password_hash="test_hash"
            )
            session.add(test_user)
            await session.commit()
            await session.refresh(test_user)
            print(f"[OK] Test user created with ID: {test_user.id}")

            # Try to create a test task
            print("Creating test task...")
            test_task = Task(
                title="Test Task",
                description="This is a test task",
                status="pending",
                user_id=test_user.id
            )
            session.add(test_task)
            await session.commit()
            await session.refresh(test_task)
            print(f"[OK] Test task created with ID: {test_task.id}")

        print("[OK] Database test completed successfully!")
        return True

    except Exception as e:
        print(f"[ERROR] Database test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    asyncio.run(test_database())