"""Contract test for the embedding process in the RAG Chatbot API."""

from unittest.mock import AsyncMock, patch

import pytest
from fastapi.testclient import TestClient

from backend.main import app
from backend.services.embedding_service import embedding_service


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    with TestClient(app) as test_client:
        yield test_client


@pytest.mark.asyncio
async def test_embedding_generation_contract_success():
    """Test that the embedding service follows the expected contract for successful embedding generation."""
    # Test the embedding service directly
    test_text = "This is a test sentence for embedding."

    with patch.object(
        embedding_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_generate:
        mock_embedding = [0.1, 0.2, 0.3, 0.4, 0.5]  # Simplified embedding
        mock_generate.return_value = mock_embedding

        # Call the embedding service
        result = await embedding_service.generate_embedding(test_text)

        # Verify the result structure and types
        assert isinstance(result, list)
        assert len(result) > 0
        assert all(isinstance(val, float) for val in result)

        # Verify that Cohere service was called
        mock_generate.assert_called_once()


@pytest.mark.asyncio
async def test_embedding_batch_generation_contract_success():
    """Test that the embedding service follows the expected contract for batch embedding generation."""
    # Test the embedding service directly
    test_texts = [
        "This is the first test sentence.",
        "This is the second test sentence.",
        "This is the third test sentence.",
    ]

    with patch.object(
        embedding_service.cohere_service,
        "generate_embeddings_batch",
        new_callable=AsyncMock,
    ) as mock_generate_batch:
        mock_embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9]]
        mock_generate_batch.return_value = mock_embeddings

        # Call the embedding service
        results = await embedding_service.generate_embeddings_batch(test_texts)

        # Verify the result structure and types
        assert isinstance(results, list)
        assert len(results) == len(test_texts)
        for embedding in results:
            assert isinstance(embedding, list)
            assert len(embedding) > 0
            assert all(isinstance(val, float) for val in embedding)

        # Verify that Cohere service was called
        mock_generate_batch.assert_called_once()


@pytest.mark.asyncio
async def test_create_embedding_record_contract_success():
    """Test that creating an embedding record follows the expected contract."""
    test_embedding = [0.1, 0.2, 0.3, 0.4, 0.5]
    content_id = "test_content_123"
    model_name = "embed-english-v3.0"

    with patch.object(
        embedding_service.db, "insert_embedding", new_callable=AsyncMock
    ) as mock_insert:
        with patch.object(
            embedding_service.qdrant, "store_embedding", new_callable=AsyncMock
        ) as mock_store:
            mock_insert.return_value = "test_embedding_123"
            mock_store.return_value = "test_embedding_123"

            # Call the embedding service
            result = await embedding_service.create_embedding_record(
                content_id=content_id,
                embedding=test_embedding,
                model_name=model_name,
                module="test_module",
                chapter="test_chapter",
                section="test_section",
            )

            # Verify the result structure
            assert hasattr(result, "embedding_id")
            assert hasattr(result, "content_id")
            assert hasattr(result, "embedding")
            assert hasattr(result, "model_name")
            assert hasattr(result, "module")
            assert hasattr(result, "chapter")
            assert hasattr(result, "section")
            assert hasattr(result, "metadata")
            assert hasattr(result, "created_at")
            assert hasattr(result, "updated_at")

            # Verify field values
            assert result.content_id == content_id
            assert result.model_name == model_name
            assert result.module == "test_module"
            assert result.chapter == "test_chapter"
            assert result.section == "test_section"
            assert result.embedding == test_embedding


@pytest.mark.asyncio
async def test_create_embeddings_for_content_contract_success():
    """Test that creating embeddings for content follows the expected contract."""
    # Create a mock content object
    from backend.models.content import TextbookContent

    mock_content = TextbookContent(
        content_id="test_content_123",
        title="Test Content",
        text="This is a test content for embedding. It contains multiple sentences. Each sentence should be processed properly.",
        module="test_module",
        chapter="test_chapter",
        section="test_section",
    )

    with patch.object(embedding_service, "chunk_content") as mock_chunk:
        with patch.object(
            embedding_service, "generate_embeddings_batch", new_callable=AsyncMock
        ) as mock_generate_batch:
            with patch.object(
                embedding_service, "create_embedding_record", new_callable=AsyncMock
            ) as mock_create_record:
                # Mock chunking to return 2 chunks
                from backend.models.content import ContentChunk

                mock_chunks = [
                    ContentChunk(
                        content_id="test_content_123",
                        text_chunk="This is a test content for embedding.",
                        chunk_index=0,
                        module="test_module",
                        chapter="test_chapter",
                        section="test_section",
                    ),
                    ContentChunk(
                        content_id="test_content_123",
                        text_chunk="It contains multiple sentences. Each sentence should be processed properly.",
                        chunk_index=1,
                        module="test_module",
                        chapter="test_chapter",
                        section="test_section",
                    ),
                ]
                mock_chunk.return_value = mock_chunks

                # Mock embedding generation
                mock_embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
                mock_generate_batch.return_value = mock_embeddings

                # Mock embedding record creation
                from backend.models.embedding import EmbeddingModel

                mock_embedding_records = [
                    EmbeddingModel(
                        embedding_id="emb_1",
                        content_id="test_content_123",
                        embedding=[0.1, 0.2, 0.3],
                        model_name="embed-english-v3.0",
                        module="test_module",
                        chapter="test_chapter",
                        section="test_section",
                    ),
                    EmbeddingModel(
                        embedding_id="emb_2",
                        content_id="test_content_123",
                        embedding=[0.4, 0.5, 0.6],
                        model_name="embed-english-v3.0",
                        module="test_module",
                        chapter="test_chapter",
                        section="test_section",
                    ),
                ]
                mock_create_record.side_effect = mock_embedding_records

                # Call the embedding service
                results = await embedding_service.create_embeddings_for_content(
                    mock_content
                )

                # Verify the result structure
                assert isinstance(results, list)
                assert len(results) == 2  # Should match number of chunks

                for result in results:
                    assert hasattr(result, "embedding_id")
                    assert hasattr(result, "content_id")
                    assert hasattr(result, "embedding")
                    assert result.content_id == "test_content_123"


@pytest.mark.asyncio
async def test_retrieve_similar_embeddings_contract_success():
    """Test that retrieving similar embeddings follows the expected contract."""
    query_embedding = [0.5, 0.6, 0.7]
    top_k = 3

    with patch.object(
        embedding_service.qdrant, "search_similar", new_callable=AsyncMock
    ) as mock_search:
        with patch.object(
            embedding_service.db, "get_embedding_by_id", new_callable=AsyncMock
        ) as mock_get_embedding:
            # Mock search results
            mock_search_results = [
                type(
                    "obj",
                    (object,),
                    {
                        "embedding_id": "emb_1",
                        "content_id": "content_1",
                        "relevance_score": 0.95,
                    },
                )(),
                type(
                    "obj",
                    (object,),
                    {
                        "embedding_id": "emb_2",
                        "content_id": "content_2",
                        "relevance_score": 0.87,
                    },
                )(),
                type(
                    "obj",
                    (object,),
                    {
                        "embedding_id": "emb_3",
                        "content_id": "content_3",
                        "relevance_score": 0.78,
                    },
                )(),
            ]
            mock_search.return_value = mock_search_results

            # Mock getting embedding records from DB
            from backend.models.embedding import EmbeddingModel

            mock_embedding_records = [
                EmbeddingModel(
                    embedding_id="emb_1",
                    content_id="content_1",
                    embedding=[0.1, 0.2, 0.3],
                    model_name="embed-english-v3.0",
                    module="test_module",
                    chapter="test_chapter",
                    section="test_section",
                ),
                EmbeddingModel(
                    embedding_id="emb_2",
                    content_id="content_2",
                    embedding=[0.4, 0.5, 0.6],
                    model_name="embed-english-v3.0",
                    module="test_module",
                    chapter="test_chapter",
                    section="test_section",
                ),
                EmbeddingModel(
                    embedding_id="emb_3",
                    content_id="content_3",
                    embedding=[0.7, 0.8, 0.9],
                    model_name="embed-english-v3.0",
                    module="test_module",
                    chapter="test_chapter",
                    section="test_section",
                ),
            ]
            mock_get_embedding.side_effect = mock_embedding_records

            # Call the embedding service
            results = await embedding_service.retrieve_similar_embeddings(
                query_embedding, top_k
            )

            # Verify the result structure
            assert isinstance(results, list)
            assert len(results) == 3  # Should match top_k

            for result in results:
                assert hasattr(result, "embedding_id")
                assert hasattr(result, "content_id")
                assert hasattr(result, "embedding")
                assert hasattr(result, "model_name")


@pytest.mark.asyncio
async def test_chunk_content_contract_success():
    """Test that content chunking follows the expected contract."""
    from backend.models.content import TextbookContent

    mock_content = TextbookContent(
        content_id="test_content_123",
        title="Test Content",
        text="This is a test content for embedding. It contains multiple sentences. Each sentence should be processed properly. Here is another sentence for testing purposes.",
        module="test_module",
        chapter="test_chapter",
        section="test_section",
    )

    # Call the embedding service
    chunks = embedding_service.chunk_content(mock_content)

    # Verify the result structure
    assert isinstance(chunks, list)
    assert len(chunks) > 0

    for chunk in chunks:
        assert hasattr(chunk, "content_id")
        assert hasattr(chunk, "text_chunk")
        assert hasattr(chunk, "chunk_index")
        assert hasattr(chunk, "module")
        assert hasattr(chunk, "chapter")
        assert hasattr(chunk, "section")

        # Verify field values
        assert chunk.content_id == "test_content_123"
        assert isinstance(chunk.text_chunk, str)
        assert len(chunk.text_chunk) > 0
        assert isinstance(chunk.chunk_index, int)


@pytest.mark.asyncio
async def test_embedding_error_handling():
    """Test that the embedding service properly handles errors."""
    test_text = "This is a test sentence for embedding."

    with patch.object(
        embedding_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_generate:
        mock_generate.side_effect = Exception("Cohere API error")

        # Call the embedding service and expect an exception
        with pytest.raises(Exception) as exc_info:
            await embedding_service.generate_embedding(test_text)

        # Verify the exception type or message
        assert "Cohere API error" in str(exc_info.value)


@pytest.mark.asyncio
async def test_embedding_request_validation():
    """Test that embedding requests are properly validated."""
    # Test with empty text
    with pytest.raises(Exception):
        await embedding_service.generate_embedding("")

    # Test with very long text (would be validated at the API level typically)
    long_text = "A" * 6000  # Exceeds typical limit
    with patch.object(
        embedding_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_generate:
        mock_generate.return_value = [0.1, 0.2, 0.3]
        # This should work if validation is handled by Cohere service
        result = await embedding_service.generate_embedding(long_text)
        assert isinstance(result, list)


if __name__ == "__main__":
    pytest.main([__file__])
