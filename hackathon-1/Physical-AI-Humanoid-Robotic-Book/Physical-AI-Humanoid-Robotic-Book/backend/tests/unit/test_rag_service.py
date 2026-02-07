"""Unit tests for the RAG service in the RAG Chatbot API."""

from unittest.mock import AsyncMock, patch

import pytest

from backend.models.query import QueryRequest
from backend.models.response import QueryResponse, SourceCitation
from backend.services.rag_service import RAGService


@pytest.fixture
def rag_service():
    """Create a RAG service instance for testing."""
    service = RAGService()
    return service


@pytest.mark.asyncio
async def test_rag_service_initialization(rag_service):
    """Test that the RAG service initializes correctly."""
    assert rag_service.cohere_service is not None
    assert rag_service.openai_service is not None
    assert rag_service.db is not None
    assert rag_service.qdrant is not None


@pytest.mark.asyncio
async def test_process_query_basic(rag_service):
    """Test basic query processing."""
    query_request = QueryRequest(
        query="What is Physical AI?",
        context="Introduction to Physical AI concepts",
        user_id="test_user_123",
    )

    with patch.object(
        rag_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_embed:
        with patch.object(
            rag_service.qdrant, "search_similar", new_callable=AsyncMock
        ) as mock_search:
            with patch.object(
                rag_service.openai_service, "generate_response", new_callable=AsyncMock
            ) as mock_generate:
                with patch.object(
                    rag_service.db, "save_query", new_callable=AsyncMock
                ) as mock_save_query:
                    with patch.object(
                        rag_service.db, "save_response", new_callable=AsyncMock
                    ) as mock_save_response:
                        # Mock the embedding generation
                        mock_embed.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

                        # Mock the search results
                        mock_search.return_value = [
                            {
                                "content_id": "content_001",
                                "text_chunk": "Physical AI is an approach that emphasizes physics-based reasoning.",
                                "relevance_score": 0.95,
                                "module": "001-intro-physical-ai",
                                "chapter": "1",
                                "section": "1.1",
                                "metadata": {},
                            }
                        ]

                        # Mock the response generation
                        mock_response_text = "Physical AI is an approach that emphasizes physics-based reasoning in AI systems."
                        mock_generate.return_value = mock_response_text

                        # Mock database save operations
                        mock_save_query.return_value = "query_123"
                        mock_save_response.return_value = "response_123"

                        # Call the method
                        result = await rag_service.process_query(query_request)

                        # Verify the result
                        assert isinstance(result, QueryResponse)
                        assert "physics-based reasoning" in result.answer_text
                        assert result.confidence_score > 0.5
                        assert len(result.source_citations) == 1

                        # Verify method calls
                        mock_embed.assert_called_once()
                        mock_search.assert_called_once()
                        mock_generate.assert_called_once()
                        mock_save_query.assert_called_once()
                        mock_save_response.assert_called_once()


@pytest.mark.asyncio
async def test_process_query_with_multiple_sources(rag_service):
    """Test query processing with multiple source citations."""
    query_request = QueryRequest(
        query="What are the applications of Physical AI?",
        context="Applications section",
        user_id="test_user_456",
    )

    with patch.object(
        rag_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_embed:
        with patch.object(
            rag_service.qdrant, "search_similar", new_callable=AsyncMock
        ) as mock_search:
            with patch.object(
                rag_service.openai_service, "generate_response", new_callable=AsyncMock
            ) as mock_generate:
                with patch.object(
                    rag_service.db, "save_query", new_callable=AsyncMock
                ) as mock_save_query:
                    with patch.object(
                        rag_service.db, "save_response", new_callable=AsyncMock
                    ) as mock_save_response:
                        # Mock the embedding generation
                        mock_embed.return_value = [0.5, 0.6, 0.7, 0.8, 0.9]

                        # Mock multiple search results
                        mock_search.return_value = [
                            {
                                "content_id": "content_001",
                                "text_chunk": "Physical AI has applications in robotics.",
                                "relevance_score": 0.92,
                                "module": "002-applications",
                                "chapter": "2",
                                "section": "2.1",
                                "metadata": {},
                            },
                            {
                                "content_id": "content_002",
                                "text_chunk": "Physical AI is used in automation systems.",
                                "relevance_score": 0.88,
                                "module": "002-applications",
                                "chapter": "2",
                                "section": "2.2",
                                "metadata": {},
                            },
                            {
                                "content_id": "content_003",
                                "text_chunk": "Physical AI contributes to safe operation.",
                                "relevance_score": 0.85,
                                "module": "002-applications",
                                "chapter": "2",
                                "section": "2.3",
                                "metadata": {},
                            },
                        ]

                        # Mock the response generation
                        mock_response_text = "Physical AI has applications in robotics, automation systems, and contributes to safe operation."
                        mock_generate.return_value = mock_response_text

                        # Mock database save operations
                        mock_save_query.return_value = "query_456"
                        mock_save_response.return_value = "response_456"

                        # Call the method
                        result = await rag_service.process_query(query_request)

                        # Verify the result
                        assert isinstance(result, QueryResponse)
                        assert result.confidence_score > 0.5
                        assert len(result.source_citations) == 3

                        # Verify all citations have proper structure
                        for citation in result.source_citations:
                            assert isinstance(citation, SourceCitation)
                            assert citation.content_id in [
                                "content_001",
                                "content_002",
                                "content_003",
                            ]

                        # Verify method calls
                        mock_embed.assert_called_once()
                        mock_search.assert_called_once()
                        mock_generate.assert_called_once()


@pytest.mark.asyncio
async def test_process_query_empty_results(rag_service):
    """Test query processing when no relevant results are found."""
    query_request = QueryRequest(
        query="What is unknown topic?",
        context="Unknown context",
        user_id="test_user_789",
    )

    with patch.object(
        rag_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_embed:
        with patch.object(
            rag_service.qdrant, "search_similar", new_callable=AsyncMock
        ) as mock_search:
            with patch.object(
                rag_service.openai_service, "generate_response", new_callable=AsyncMock
            ) as mock_generate:
                with patch.object(
                    rag_service.db, "save_query", new_callable=AsyncMock
                ) as mock_save_query:
                    with patch.object(
                        rag_service.db, "save_response", new_callable=AsyncMock
                    ) as mock_save_response:
                        # Mock the embedding generation
                        mock_embed.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

                        # Mock empty search results
                        mock_search.return_value = []

                        # Mock the response generation for no results
                        mock_response_text = "I couldn't find relevant information about this topic in the textbook."
                        mock_generate.return_value = mock_response_text

                        # Mock database save operations
                        mock_save_query.return_value = "query_789"
                        mock_save_response.return_value = "response_789"

                        # Call the method
                        result = await rag_service.process_query(query_request)

                        # Verify the result
                        assert isinstance(result, QueryResponse)
                        assert (
                            "couldn't find relevant information"
                            in result.answer_text.lower()
                        )
                        assert len(result.source_citations) == 0

                        # Verify method calls
                        mock_embed.assert_called_once()
                        mock_search.assert_called_once()
                        mock_generate.assert_called_once()


@pytest.mark.asyncio
async def test_process_text_selection_query(rag_service):
    """Test text selection query processing."""
    selected_text = (
        "Physical AI is an approach that emphasizes physics-based reasoning."
    )
    context = "Introduction section"

    with patch.object(
        rag_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_embed:
        with patch.object(
            rag_service.qdrant, "search_similar", new_callable=AsyncMock
        ) as mock_search:
            with patch.object(
                rag_service.openai_service, "generate_response", new_callable=AsyncMock
            ) as mock_generate:
                with patch.object(
                    rag_service.db, "save_query", new_callable=AsyncMock
                ) as mock_save_query:
                    with patch.object(
                        rag_service.db, "save_response", new_callable=AsyncMock
                    ) as mock_save_response:
                        # Mock the embedding generation
                        mock_embed.return_value = [0.3, 0.4, 0.5, 0.6, 0.7]

                        # Mock the search results
                        mock_search.return_value = [
                            {
                                "content_id": "content_001",
                                "text_chunk": "Physical AI is an approach that emphasizes physics-based reasoning.",
                                "relevance_score": 0.98,
                                "module": "001-intro-physical-ai",
                                "chapter": "1",
                                "section": "1.1",
                                "metadata": {},
                            }
                        ]

                        # Mock the response generation
                        mock_response_text = "The selected text introduces the concept of Physical AI, which emphasizes physics-based reasoning in AI systems."
                        mock_generate.return_value = mock_response_text

                        # Mock database save operations
                        mock_save_query.return_value = "query_text_sel"
                        mock_save_response.return_value = "response_text_sel"

                        # Call the method
                        result = await rag_service.process_text_selection_query(
                            selected_text, context
                        )

                        # Verify the result
                        assert isinstance(result, QueryResponse)
                        assert "introduces the concept" in result.answer_text
                        assert result.query_text == selected_text
                        assert len(result.source_citations) == 1

                        # Verify method calls
                        mock_embed.assert_called_once()
                        mock_search.assert_called_once()
                        mock_generate.assert_called_once()


@pytest.mark.asyncio
async def test_rag_service_embedding_error_handling(rag_service):
    """Test RAG service error handling when embedding fails."""
    query_request = QueryRequest(
        query="What is Physical AI?", context="Introduction", user_id="test_user_error"
    )

    with patch.object(
        rag_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_embed:
        mock_embed.side_effect = Exception("Embedding API error")

        # Call the method and expect an exception
        with pytest.raises(Exception) as exc_info:
            await rag_service.process_query(query_request)

        # Verify the exception
        assert "Embedding API error" in str(exc_info.value)


@pytest.mark.asyncio
async def test_rag_service_search_error_handling(rag_service):
    """Test RAG service error handling when search fails."""
    query_request = QueryRequest(
        query="What is Physical AI?", context="Introduction", user_id="test_user_error"
    )

    with patch.object(
        rag_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_embed:
        with patch.object(
            rag_service.qdrant, "search_similar", new_callable=AsyncMock
        ) as mock_search:
            mock_embed.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]
            mock_search.side_effect = Exception("Search API error")

            # Call the method and expect an exception
            with pytest.raises(Exception) as exc_info:
                await rag_service.process_query(query_request)

            # Verify the exception
            assert "Search API error" in str(exc_info.value)


@pytest.mark.asyncio
async def test_calculate_confidence_score(rag_service):
    """Test confidence score calculation."""
    # Test with high relevance scores
    citations = [
        SourceCitation(
            content_id="content_1",
            title="Title 1",
            text_excerpt="Excerpt 1",
            module="module_1",
            chapter="ch_1",
            section="sec_1",
            relevance_score=0.95,
        ),
        SourceCitation(
            content_id="content_2",
            title="Title 2",
            text_excerpt="Excerpt 2",
            module="module_1",
            chapter="ch_1",
            section="sec_1",
            relevance_score=0.88,
        ),
    ]

    confidence = rag_service._calculate_confidence_score(citations)
    assert 0 <= confidence <= 1
    assert confidence > 0.8  # Should be high with good relevance scores

    # Test with low relevance scores
    low_citations = [
        SourceCitation(
            content_id="content_1",
            title="Title 1",
            text_excerpt="Excerpt 1",
            module="module_1",
            chapter="ch_1",
            section="sec_1",
            relevance_score=0.2,
        )
    ]

    low_confidence = rag_service._calculate_confidence_score(low_citations)
    assert low_confidence < 0.5  # Should be low with poor relevance scores

    # Test with no citations
    no_confidence = rag_service._calculate_confidence_score([])
    assert no_confidence < 0.3  # Should be low with no citations


@pytest.mark.asyncio
async def test_format_response_with_citations(rag_service):
    """Test response formatting with citations."""
    query_text = "What is Physical AI?"
    answer_text = "Physical AI is an approach that emphasizes physics-based reasoning."
    citations = [
        SourceCitation(
            content_id="content_001",
            title="Introduction to Physical AI",
            text_excerpt="Physical AI is an approach that emphasizes physics-based reasoning.",
            module="001-intro-physical-ai",
            chapter="1",
            section="1.1",
            relevance_score=0.95,
        )
    ]

    response = rag_service._format_response(query_text, answer_text, citations)

    assert isinstance(response, QueryResponse)
    assert response.query_text == query_text
    assert response.answer_text == answer_text
    assert len(response.source_citations) == 1
    assert response.source_citations[0].content_id == "content_001"
    assert response.confidence_score > 0.5


if __name__ == "__main__":
    pytest.main([__file__])
