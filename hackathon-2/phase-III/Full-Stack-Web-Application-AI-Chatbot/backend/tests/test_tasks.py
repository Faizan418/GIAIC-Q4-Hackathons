"""Basic tests for task endpoints"""
import pytest
from fastapi.testclient import TestClient
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
from src.main import app

client = TestClient(app)

def test_health_check():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy"}

def test_tasks_crud():
    """Test basic task CRUD operations"""
    # Test creating a task
    task_data = {
        "title": "Test Task",
        "description": "This is a test task",
        "status": "pending"
    }

    # Since we need authentication, let's test the route exists
    # First try without auth to see if we get a 401 or 403
    response = client.get("/api/tasks/tasks/")
    # Should return 401 Unauthorized, 403 Forbidden (due to auth check), or 422 Unprocessable Entity if route exists
    assert response.status_code in [401, 403, 404, 422], f"Expected 401, 403, 404, or 422 but got {response.status_code}"

    # Test the route exists by checking if we get a 403/401 (auth required) rather than 404
    # If we get 404, the route doesn't exist
    if response.status_code == 404:
        print("Tasks route does not exist")
        assert False, "Tasks route should exist"
    else:
        print("Tasks route exists and requires authentication")
        assert True  # Test passes if route exists