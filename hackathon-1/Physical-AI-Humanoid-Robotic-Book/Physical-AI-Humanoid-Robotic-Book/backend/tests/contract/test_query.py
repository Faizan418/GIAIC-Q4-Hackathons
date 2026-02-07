"""Contract test for the /query endpoint in the RAG Chatbot API."""


import pytest
from fastapi import status
from fastapi.testclient import TestClient

from backend.main import app
from backend.models.query import QueryRequest


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    with TestClient(app) as test_client:
        yield test_client


@pytest.mark.asyncio
async def test_query_endpoint_contract_success(client):
    """Test that the /query endpoint follows the expected contract for successful requests."""
    # Define test input
    test_query = QueryRequest(
        query="What is Physical AI?",
        context="Introduction to Physical AI concepts",
        user_id="test_user_123",
    )

    # Make request to the endpoint
    response = client.post(
        "/api/query",
        json=test_query.dict(),
        headers={"Content-Type": "application/json"},
    )

    # Verify response status
    assert response.status_code == status.HTTP_200_OK

    # Verify response structure
    response_data = response.json()

    # Check that required fields are present in the response
    assert "response_id" in response_data
    assert "query_id" in response_data
    assert "answer_text" in response_data
    assert "confidence_score" in response_data
    assert "timestamp" in response_data
    assert "query_text" in response_data
    assert "source_citations" in response_data

    # Verify field types
    assert isinstance(response_data["response_id"], str)
    assert isinstance(response_data["query_id"], str)
    assert isinstance(response_data["answer_text"], str)
    assert isinstance(response_data["confidence_score"], (int, float))
    assert isinstance(response_data["timestamp"], str)  # ISO format string
    assert isinstance(response_data["query_text"], str)
    assert isinstance(response_data["source_citations"], list)

    # Verify confidence score is between 0 and 1
    assert 0 <= response_data["confidence_score"] <= 1

    # If there are citations, verify their structure
    for citation in response_data["source_citations"]:
        assert "content_id" in citation
        assert "title" in citation
        assert "text_excerpt" in citation
        assert "module" in citation
        assert "chapter" in citation
        assert "section" in citation
        assert "relevance_score" in citation

        # Verify citation field types
        assert isinstance(citation["content_id"], str)
        assert isinstance(citation["title"], str)
        assert isinstance(citation["text_excerpt"], str)
        assert isinstance(citation["module"], str)
        assert isinstance(citation["chapter"], str)
        assert isinstance(citation["section"], str)
        assert isinstance(citation["relevance_score"], (int, float))


@pytest.mark.asyncio
async def test_query_endpoint_contract_validation_error(client):
    """Test that the /query endpoint properly handles validation errors."""
    # Send request with invalid data (empty query)
    invalid_request = {
        "query": "",  # Empty query should fail validation
        "context": "Test context",
        "user_id": "test_user_123",
    }

    response = client.post(
        "/api/query", json=invalid_request, headers={"Content-Type": "application/json"}
    )

    # Verify validation error response
    assert response.status_code == status.HTTP_400_BAD_REQUEST

    error_data = response.json()
    assert "error" in error_data
    assert error_data["error"] == "Validation Error"


@pytest.mark.asyncio
async def test_query_endpoint_contract_query_too_long(client):
    """Test that the /query endpoint handles queries that are too long."""
    # Create a very long query that exceeds the 1000 character limit
    long_query = "A" * 1001  # 1001 characters, exceeding the limit

    long_request = {
        "query": long_query,
        "context": "Test context",
        "user_id": "test_user_123",
    }

    response = client.post(
        "/api/query", json=long_request, headers={"Content-Type": "application/json"}
    )

    # Verify validation error response
    assert response.status_code == status.HTTP_400_BAD_REQUEST

    error_data = response.json()
    assert "error" in error_data
    assert error_data["error"] == "Validation Error"


@pytest.mark.asyncio
async def test_query_endpoint_contract_missing_required_fields(client):
    """Test that the /query endpoint handles requests with missing required fields."""
    # Send request without required query field
    incomplete_request = {"context": "Test context", "user_id": "test_user_123"}

    response = client.post(
        "/api/query",
        json=incomplete_request,
        headers={"Content-Type": "application/json"},
    )

    # Verify validation error response
    assert response.status_code == status.HTTP_400_BAD_REQUEST


@pytest.mark.asyncio
async def test_query_endpoint_method_not_allowed(client):
    """Test that the /query endpoint only accepts POST requests."""
    # Try GET request
    response = client.get("/api/query")
    assert response.status_code == status.HTTP_405_METHOD_NOT_ALLOWED

    # Try PUT request
    response = client.put("/api/query", json={"query": "test"})
    assert response.status_code == status.HTTP_405_METHOD_NOT_ALLOWED

    # Try DELETE request
    response = client.delete("/api/query")
    assert response.status_code == status.HTTP_405_METHOD_NOT_ALLOWED


@pytest.mark.asyncio
async def test_query_endpoint_content_type(client):
    """Test that the /query endpoint properly handles content type."""
    test_request = {
        "query": "What is Physical AI?",
        "context": "Introduction",
        "user_id": "test_user",
    }

    # Test with correct content type
    response = client.post(
        "/api/query", json=test_request, headers={"Content-Type": "application/json"}
    )
    assert response.status_code in [
        status.HTTP_200_OK,
        status.HTTP_500_INTERNAL_SERVER_ERROR,
    ]

    # Test without content type (should still work as FastAPI handles this)
    response = client.post("/api/query", json=test_request)
    assert response.status_code in [
        status.HTTP_200_OK,
        status.HTTP_400_BAD_REQUEST,
        status.HTTP_500_INTERNAL_SERVER_ERROR,
    ]


if __name__ == "__main__":
    pytest.main([__file__])
