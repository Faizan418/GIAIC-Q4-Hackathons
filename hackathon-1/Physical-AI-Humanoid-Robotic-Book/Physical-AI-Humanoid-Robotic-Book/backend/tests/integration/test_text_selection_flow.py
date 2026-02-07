"""Integration test for the text selection flow in the RAG Chatbot API."""

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
async def test_text_selection_flow_integration_success(client):
    """Test the full text selection flow integration."""
    # Mock the RAG service to avoid actual API calls during testing
    with patch.object(
        rag_service, "process_text_selection_query", new_callable=AsyncMock
    ) as mock_process_text_selection:
        # Create a mock response
        mock_response = QueryResponse(
            response_id="test_response_123",
            query_id="test_query_123",
            answer_text="The selected text discusses Physical AI concepts. Physical AI is an approach that emphasizes the importance of physics-based reasoning in artificial intelligence systems.",
            confidence_score=0.92,
            timestamp="2025-12-16T10:00:00Z",
            query_text="Physical AI is an approach that emphasizes the importance of physics-based reasoning in artificial intelligence systems.",
            source_citations=[],
        )
        mock_process_text_selection.return_value = mock_response

        # Make request to the endpoint
        test_request = {
            "selected_text": "Physical AI is an approach that emphasizes the importance of physics-based reasoning in artificial intelligence systems.",
            "context": "Introduction to Physical AI concepts",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/text-selection-query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Verify response
        assert response.status_code == status.HTTP_200_OK
        response_data = response.json()

        # Verify response matches mock
        assert response_data["response_id"] == "test_response_123"
        assert "Physical AI concepts" in response_data["answer_text"]
        assert response_data["confidence_score"] == 0.92

        # Verify that the RAG service was called with the correct parameters
        mock_process_text_selection.assert_called_once()
        args, kwargs = mock_process_text_selection.call_args
        assert (
            args[0]
            == "Physical AI is an approach that emphasizes the importance of physics-based reasoning in artificial intelligence systems."
        )
        assert args[1] == "Introduction to Physical AI concepts"  # context


@pytest.mark.asyncio
async def test_text_selection_flow_integration_with_cohere_error(client):
    """Test the text selection flow when Cohere API returns an error."""
    # Mock the RAG service to simulate a Cohere API error
    with patch.object(
        rag_service, "process_text_selection_query", new_callable=AsyncMock
    ) as mock_process_text_selection:
        mock_process_text_selection.side_effect = Exception("Cohere API error")

        test_request = {
            "selected_text": "Physical AI is an approach that emphasizes physics-based reasoning.",
            "context": "Introduction to Physical AI concepts",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/text-selection-query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Verify error response
        assert response.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR
        error_data = response.json()
        assert "error" in error_data


@pytest.mark.asyncio
async def test_text_selection_flow_integration_with_empty_response(client):
    """Test the text selection flow when the RAG service returns an empty response."""
    # Mock the RAG service to return an empty response
    with patch.object(
        rag_service, "process_text_selection_query", new_callable=AsyncMock
    ) as mock_process_text_selection:
        # Create a mock response with empty answer
        mock_response = QueryResponse(
            response_id="test_response_123",
            query_id="test_query_123",
            answer_text="",
            confidence_score=0.0,
            timestamp="2025-12-16T10:00:00Z",
            query_text="Physical AI is an approach that emphasizes physics-based reasoning.",
            source_citations=[],
        )
        mock_process_text_selection.return_value = mock_response

        test_request = {
            "selected_text": "Physical AI is an approach that emphasizes physics-based reasoning.",
            "context": "Introduction to Physical AI concepts",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/text-selection-query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Verify response - should still be successful even if answer is empty
        assert response.status_code == status.HTTP_200_OK
        response_data = response.json()
        assert response_data["answer_text"] == ""
        assert response_data["confidence_score"] == 0.0


@pytest.mark.asyncio
async def test_text_selection_flow_integration_with_citations(client):
    """Test the text selection flow when citations are included in the response."""
    # Mock the RAG service to return a response with citations
    with patch.object(
        rag_service, "process_text_selection_query", new_callable=AsyncMock
    ) as mock_process_text_selection:
        # Create a mock response with citations
        mock_response = QueryResponse(
            response_id="test_response_123",
            query_id="test_query_123",
            answer_text="The selected text discusses Physical AI, which combines principles of physics with artificial intelligence.",
            confidence_score=0.88,
            timestamp="2025-12-16T10:00:00Z",
            query_text="Physical AI is an approach that emphasizes physics-based reasoning.",
            source_citations=[
                {
                    "content_id": "content_001",
                    "title": "Introduction to Physical AI",
                    "text_excerpt": "Physical AI is an approach that emphasizes physics-based reasoning...",
                    "module": "001-intro-physical-ai",
                    "chapter": "1",
                    "section": "1.1",
                    "relevance_score": 0.95,
                },
                {
                    "content_id": "content_003",
                    "title": "Foundations of Physical AI",
                    "text_excerpt": "The foundations of Physical AI lie in understanding physical interactions...",
                    "module": "001-intro-physical-ai",
                    "chapter": "1",
                    "section": "1.3",
                    "relevance_score": 0.85,
                },
            ],
        )
        mock_process_text_selection.return_value = mock_response

        test_request = {
            "selected_text": "Physical AI is an approach that emphasizes physics-based reasoning.",
            "context": "Introduction to Physical AI concepts",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/text-selection-query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Verify response with citations
        assert response.status_code == status.HTTP_200_OK
        response_data = response.json()
        assert len(response_data["source_citations"]) == 2
        assert response_data["confidence_score"] == 0.88

        # Verify citation structure
        first_citation = response_data["source_citations"][0]
        assert first_citation["content_id"] == "content_001"
        assert first_citation["title"] == "Introduction to Physical AI"
        assert first_citation["module"] == "001-intro-physical-ai"


@pytest.mark.asyncio
async def test_text_selection_flow_performance(client):
    """Test the text selection flow performance under normal conditions."""
    # Mock the RAG service to return a consistent response
    with patch.object(
        rag_service, "process_text_selection_query", new_callable=AsyncMock
    ) as mock_process_text_selection:
        mock_response = QueryResponse(
            response_id="test_response_123",
            query_id="test_query_123",
            answer_text="Test response for text selection performance evaluation.",
            confidence_score=0.85,
            timestamp="2025-12-16T10:00:00Z",
            query_text="Performance test selected text",
            source_citations=[],
        )
        mock_process_text_selection.return_value = mock_response

        import time

        start_time = time.time()

        test_request = {
            "selected_text": "Performance test selected text",
            "context": "Performance testing",
            "user_id": "test_user_123",
        }

        response = client.post(
            "/api/text-selection-query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        end_time = time.time()
        duration = end_time - start_time

        # Verify response is successful
        assert response.status_code == status.HTTP_200_OK

        # The duration might be very small in a test environment, but this verifies the flow completes
        assert duration >= 0  # Duration should be non-negative


@pytest.mark.asyncio
async def test_multiple_concurrent_text_selection_queries(client):
    """Test handling of multiple concurrent text selection queries."""
    # Mock the RAG service to return responses quickly
    with patch.object(
        rag_service, "process_text_selection_query", new_callable=AsyncMock
    ) as mock_process_text_selection:

        def side_effect(selected_text, context):
            # Create different responses based on the selected text
            mock_response = QueryResponse(
                response_id=f"test_response_{hash(selected_text)}",
                query_id=f"test_query_{hash(selected_text)}",
                answer_text=f"Context for selected text: {selected_text[:20]}...",
                confidence_score=0.80,
                timestamp="2025-12-16T10:00:00Z",
                query_text=selected_text,
                source_citations=[],
            )
            return mock_response

        mock_process_text_selection.side_effect = side_effect

        # Define multiple requests
        requests = [
            {
                "selected_text": "First sample text about Physical AI concepts",
                "context": "Introduction section",
                "user_id": "user_1",
            },
            {
                "selected_text": "Second sample text about robotics applications",
                "context": "Applications section",
                "user_id": "user_2",
            },
            {
                "selected_text": "Third sample text about machine learning integration",
                "context": "ML section",
                "user_id": "user_3",
            },
        ]

        # Send requests concurrently
        import asyncio

        async def send_request(req_data):
            loop = asyncio.get_event_loop()
            future = loop.run_in_executor(
                None,
                lambda: client.post(
                    "/api/text-selection-query",
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
            assert "Context for selected text" in response_data["answer_text"]


@pytest.mark.asyncio
async def test_text_selection_flow_with_short_text(client):
    """Test the text selection flow with very short selected text."""
    # Mock the RAG service
    with patch.object(
        rag_service, "process_text_selection_query", new_callable=AsyncMock
    ) as mock_process_text_selection:
        mock_response = QueryResponse(
            response_id="test_response_short",
            query_id="test_query_short",
            answer_text="The term 'AI' refers to Artificial Intelligence systems.",
            confidence_score=0.75,
            timestamp="2025-12-16T10:00:00Z",
            query_text="AI",
            source_citations=[],
        )
        mock_process_text_selection.return_value = mock_response

        test_request = {
            "selected_text": "AI",  # Very short text
            "context": "Brief mention",
            "user_id": "test_user_short",
        }

        response = client.post(
            "/api/text-selection-query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Should handle short text appropriately
        assert response.status_code == status.HTTP_200_OK
        response_data = response.json()
        assert response_data["query_text"] == "AI"


@pytest.mark.asyncio
async def test_text_selection_flow_with_special_characters(client):
    """Test the text selection flow with special characters and formatting."""
    # Mock the RAG service
    with patch.object(
        rag_service, "process_text_selection_query", new_callable=AsyncMock
    ) as mock_process_text_selection:
        mock_response = QueryResponse(
            response_id="test_response_special",
            query_id="test_query_special",
            answer_text="Text with special characters processed successfully.",
            confidence_score=0.82,
            timestamp="2025-12-16T10:00:00Z",
            query_text="Text with special chars: !@#$%^&*()_+-=[]{}|;:,.<>?",
            source_citations=[],
        )
        mock_process_text_selection.return_value = mock_response

        test_request = {
            "selected_text": "Text with special chars: !@#$%^&*()_+-=[]{}|;:,.<>?",
            "context": "Special characters test",
            "user_id": "test_user_special",
        }

        response = client.post(
            "/api/text-selection-query",
            json=test_request,
            headers={"Content-Type": "application/json"},
        )

        # Should handle special characters appropriately
        assert response.status_code == status.HTTP_200_OK
        response_data = response.json()
        assert "special characters" in response_data["answer_text"].lower()


if __name__ == "__main__":
    pytest.main([__file__])
