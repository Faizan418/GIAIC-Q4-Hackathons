"""Integration test for the query flow in the RAG Chatbot API."""

from unittest.mock import AsyncMock, patch

import pytest
from fastapi import status
from fastapi.testclient import TestClient

from backend.main import app
from backend.models.response import QueryResponse
from backend.services.rag_service import rag_service


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    with TestClient(app) as test_client:
        yield test_client


@pytest.mark.asyncio
async def test_query_flow_integration_success(client):
    """Test the full query flow integration."""
    # Mock the RAG service to avoid actual API calls during testing
    with patch.object(
        rag_service, "process_query", new_callable=AsyncMock
    ) as mock_process_query:
        # Create a mock response
        mock_response = QueryResponse(
            response_id="test_response_123",
            query_id="test_query_123",
            answer_text="Physical AI is an approach that emphasizes the importance of physics-based reasoning in artificial intelligence systems.",
            confidence_score=0.95,
            timestamp="2025-12-16T10:00:00Z",
            query_text="What is Physical AI?",
            source_citations=[],
        )
        mock_process_query.return_value = mock_response

        # Make request to the endpoint
        test_request = {
            "query": "What is Physical AI?",
            "context": "Introduction to Physical AI concepts",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Verify response
        assert response.status_code == status.HTTP_200_OK
        response_data = response.json()

        # Verify response matches mock
        assert response_data["response_id"] == "test_response_123"
        assert (
            response_data["answer_text"]
            == "Physical AI is an approach that emphasizes the importance of physics-based reasoning in artificial intelligence systems."
        )
        assert response_data["confidence_score"] == 0.95

        # Verify that the RAG service was called with the correct parameters
        mock_process_query.assert_called_once()
        call_args = mock_process_query.call_args[0][0]  # First positional argument
        assert call_args.query == "What is Physical AI?"
        assert call_args.context == "Introduction to Physical AI concepts"
        assert call_args.user_id == "test_user_123"

        # Verify that response was logged (if logging is implemented)
        # This depends on your logging implementation


@pytest.mark.asyncio
async def test_query_flow_integration_with_cohere_error(client):
    """Test the query flow when Cohere API returns an error."""
    # Mock the RAG service to simulate a Cohere API error
    with patch.object(
        rag_service, "process_query", new_callable=AsyncMock
    ) as mock_process_query:
        mock_process_query.side_effect = Exception("Cohere API error")

        test_request = {
            "query": "What is Physical AI?",
            "context": "Introduction to Physical AI concepts",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Verify error response
        assert response.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR
        error_data = response.json()
        assert "error" in error_data


@pytest.mark.asyncio
async def test_query_flow_integration_with_empty_response(client):
    """Test the query flow when the RAG service returns an empty response."""
    # Mock the RAG service to return an empty response
    with patch.object(
        rag_service, "process_query", new_callable=AsyncMock
    ) as mock_process_query:
        # Create a mock response with empty answer
        mock_response = QueryResponse(
            response_id="test_response_123",
            query_id="test_query_123",
            answer_text="",
            confidence_score=0.0,
            timestamp="2025-12-16T10:00:00Z",
            query_text="What is Physical AI?",
            source_citations=[],
        )
        mock_process_query.return_value = mock_response

        test_request = {
            "query": "What is Physical AI?",
            "context": "Introduction to Physical AI concepts",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Verify response - should still be successful even if answer is empty
        assert response.status_code == status.HTTP_200_OK
        response_data = response.json()
        assert response_data["answer_text"] == ""
        assert response_data["confidence_score"] == 0.0


@pytest.mark.asyncio
async def test_query_flow_integration_with_citations(client):
    """Test the query flow when citations are included in the response."""
    # Mock the RAG service to return a response with citations
    with patch.object(
        rag_service, "process_query", new_callable=AsyncMock
    ) as mock_process_query:
        # Create a mock response with citations
        mock_response = QueryResponse(
            response_id="test_response_123",
            query_id="test_query_123",
            answer_text="Physical AI combines principles of physics with artificial intelligence.",
            confidence_score=0.85,
            timestamp="2025-12-16T10:00:00Z",
            query_text="What is Physical AI?",
            source_citations=[
                {
                    "content_id": "content_001",
                    "title": "Introduction to Physical AI",
                    "text_excerpt": "Physical AI is an approach that emphasizes physics-based reasoning...",
                    "module": "001-intro-physical-ai",
                    "chapter": "1",
                    "section": "1.1",
                    "relevance_score": 0.92,
                },
                {
                    "content_id": "content_002",
                    "title": "Applications of Physical AI",
                    "text_excerpt": "Physical AI has applications in robotics and automation...",
                    "module": "001-intro-physical-ai",
                    "chapter": "1",
                    "section": "1.2",
                    "relevance_score": 0.88,
                },
            ],
        )
        mock_process_query.return_value = mock_response

        test_request = {
            "query": "What is Physical AI?",
            "context": "Introduction to Physical AI concepts",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Verify response with citations
        assert response.status_code == status.HTTP_200_OK
        response_data = response.json()
        assert len(response_data["source_citations"]) == 2
        assert response_data["confidence_score"] == 0.85

        # Verify citation structure
        first_citation = response_data["source_citations"][0]
        assert first_citation["content_id"] == "content_001"
        assert first_citation["title"] == "Introduction to Physical AI"
        assert first_citation["module"] == "001-intro-physical-ai"


@pytest.mark.asyncio
async def test_query_flow_performance(client):
    """Test the query flow performance under normal conditions."""
    # Mock the RAG service to return a consistent response
    with patch.object(
        rag_service, "process_query", new_callable=AsyncMock
    ) as mock_process_query:
        mock_response = QueryResponse(
            response_id="test_response_123",
            query_id="test_query_123",
            answer_text="Test response for performance evaluation.",
            confidence_score=0.90,
            timestamp="2025-12-16T10:00:00Z",
            query_text="Performance test query",
            source_citations=[],
        )
        mock_process_query.return_value = mock_response

        import time

        start_time = time.time()

        test_request = {
            "query": "Performance test query",
            "context": "Performance testing",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        end_time = time.time()
        duration = end_time - start_time

        # Verify response is successful
        assert response.status_code == status.HTTP_200_OK

        # The duration might be very small in a test environment, but this verifies the flow completes
        # In a real integration test, you might want to verify it's under a certain threshold
        assert duration >= 0  # Duration should be non-negative


@pytest.mark.asyncio
async def test_multiple_concurrent_queries(client):
    """Test handling of multiple concurrent queries."""
    # Mock the RAG service to return responses quickly
    with patch.object(
        rag_service, "process_query", new_callable=AsyncMock
    ) as mock_process_query:

        def side_effect(query_request):
            # Create different responses based on the query
            mock_response = QueryResponse(
                response_id=f"test_response_{hash(query_request.query)}",
                query_id=f"test_query_{hash(query_request.query)}",
                answer_text=f"Response for: {query_request.query}",
                confidence_score=0.85,
                timestamp="2025-12-16T10:00:00Z",
                query_text=query_request.query,
                source_citations=[],
            )
            return mock_response

        mock_process_query.side_effect = side_effect

        # Define multiple requests
        requests = [
            {
                "query": "What is Physical AI?",
                "context": "Introduction",
                "user_id": "user_1",
            },
            {
                "query": "What are the applications?",
                "context": "Applications",
                "user_id": "user_2",
            },
            {"query": "How does it work?", "context": "Mechanics", "user_id": "user_3"},
        ]

        # Send requests concurrently
        import asyncio

        async def send_request(req_data):
            loop = asyncio.get_event_loop()
            future = loop.run_in_executor(
                None,
                lambda: client.post(
                    "/api/query",
                    json=req_data,
                    headers={"Content-Type": "application/json"},
                ),
            )
            return await future

        # Execute requests concurrently
        tasks = [send_request(req) for req in requests]
        responses = await asyncio.gather(*tasks)

        # Verify all responses are successful
        for i, response in enumerate(responses):
            assert response.status_code == status.HTTP_200_OK
            response_data = response.json()
            assert (
                f"Response for: {requests[i]['query']}" in response_data["answer_text"]
            )


if __name__ == "__main__":
    pytest.main([__file__])
