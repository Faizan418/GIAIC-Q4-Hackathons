"""Unit tests for the Embedding service in the RAG Chatbot API."""

from unittest.mock import AsyncMock, patch

import pytest

from backend.models.content import ContentChunk, TextbookContent
from backend.models.embedding import EmbeddingModel
from backend.services.embedding_service import EmbeddingService


@pytest.fixture
def embedding_service():
    """Create an embedding service instance for testing."""
    service = EmbeddingService()
    return service


@pytest.mark.asyncio
async def test_embedding_service_initialization(embedding_service):
    """Test that the embedding service initializes correctly."""
    assert embedding_service.cohere_service is not None
    assert embedding_service.db is not None
    assert embedding_service.qdrant is not None


@pytest.mark.asyncio
async def test_generate_embedding_success(embedding_service):
    """Test successful embedding generation."""
    test_text = "This is a test sentence for embedding."

    with patch.object(
        embedding_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_generate:
        expected_embedding = [0.1, 0.2, 0.3, 0.4, 0.5]
        mock_generate.return_value = expected_embedding

        result = await embedding_service.generate_embedding(test_text)

        assert result == expected_embedding
        mock_generate.assert_called_once_with(
            test_text, "embed-english-v3.0", "search_document"
        )


@pytest.mark.asyncio
async def test_generate_embeddings_batch_success(embedding_service):
    """Test successful batch embedding generation."""
    test_texts = ["Text 1", "Text 2", "Text 3"]

    with patch.object(
        embedding_service.cohere_service,
        "generate_embeddings_batch",
        new_callable=AsyncMock,
    ) as mock_generate_batch:
        expected_embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9]]
        mock_generate_batch.return_value = expected_embeddings

        result = await embedding_service.generate_embeddings_batch(test_texts)

        assert result == expected_embeddings
        assert len(result) == len(test_texts)
        mock_generate_batch.assert_called_once_with(
            test_texts, "embed-english-v3.0", "search_document"
        )


@pytest.mark.asyncio
async def test_create_embedding_record_success(embedding_service):
    """Test successful creation of an embedding record."""
    content_id = "test_content_123"
    embedding = [0.1, 0.2, 0.3, 0.4, 0.5]
    model_name = "embed-english-v3.0"

    with patch.object(
        embedding_service.db, "insert_embedding", new_callable=AsyncMock
    ) as mock_insert:
        with patch.object(
            embedding_service.qdrant, "store_embedding", new_callable=AsyncMock
        ) as mock_store:
            mock_insert.return_value = "test_embedding_123"
            mock_store.return_value = "test_embedding_123"

            result = await embedding_service.create_embedding_record(
                content_id=content_id,
                embedding=embedding,
                model_name=model_name,
                module="test_module",
                chapter="test_chapter",
                section="test_section",
                metadata={"chunk_index": 0},
            )

            assert isinstance(result, EmbeddingModel)
            assert result.content_id == content_id
            assert result.embedding == embedding
            assert result.model_name == model_name
            assert result.module == "test_module"
            assert result.chapter == "test_chapter"
            assert result.section == "test_section"
            assert result.metadata == {"chunk_index": 0}

            mock_insert.assert_called_once()
            mock_store.assert_called_once()


@pytest.mark.asyncio
async def test_create_embeddings_for_content_success(embedding_service):
    """Test successful creation of embeddings for content."""
    content = TextbookContent(
        content_id="test_content_123",
        title="Test Content",
        text="This is a test content. It has multiple sentences. Each sentence should be processed.",
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
                # Mock content chunking
                chunks = [
                    ContentChunk(
                        content_id="test_content_123",
                        text_chunk="This is a test content.",
                        chunk_index=0,
                        module="test_module",
                        chapter="test_chapter",
                        section="test_section",
                    ),
                    ContentChunk(
                        content_id="test_content_123",
                        text_chunk="It has multiple sentences.",
                        chunk_index=1,
                        module="test_module",
                        chapter="test_chapter",
                        section="test_section",
                    ),
                    ContentChunk(
                        content_id="test_content_123",
                        text_chunk="Each sentence should be processed.",
                        chunk_index=2,
                        module="test_module",
                        chapter="test_chapter",
                        section="test_section",
                    ),
                ]
                mock_chunk.return_value = chunks

                # Mock batch embedding generation
                embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9]]
                mock_generate_batch.return_value = embeddings

                # Mock embedding record creation
                embedding_models = []
                for i, (chunk, emb) in enumerate(zip(chunks, embeddings)):
                    embedding_model = EmbeddingModel(
                        embedding_id=f"emb_{i}",
                        content_id=chunk.content_id,
                        embedding=emb,
                        model_name="embed-english-v3.0",
                        module=chunk.module,
                        chapter=chunk.chapter,
                        section=chunk.section,
                    )
                    embedding_models.append(embedding_model)

                mock_create_record.side_effect = embedding_models

                # Call the method
                result = await embedding_service.create_embeddings_for_content(content)

                # Verify the result
                assert isinstance(result, list)
                assert len(result) == 3  # Should match number of chunks
                for emb_model in result:
                    assert isinstance(emb_model, EmbeddingModel)
                    assert emb_model.content_id == "test_content_123"

                # Verify method calls
                mock_chunk.assert_called_once_with(content)
                mock_generate_batch.assert_called_once()
                assert mock_create_record.call_count == 3


@pytest.mark.asyncio
async def test_chunk_content_basic(embedding_service):
    """Test basic content chunking."""
    content = TextbookContent(
        content_id="test_content_123",
        title="Test Content",
        text="This is the first sentence. This is the second sentence. This is the third sentence.",
        module="test_module",
        chapter="test_chapter",
        section="test_section",
    )

    chunks = embedding_service.chunk_content(content, max_chunk_size=30, overlap=5)

    assert isinstance(chunks, list)
    assert len(chunks) > 0

    for chunk in chunks:
        assert isinstance(chunk, ContentChunk)
        assert chunk.content_id == "test_content_123"
        assert len(chunk.text_chunk) > 0
        assert isinstance(chunk.chunk_index, int)


@pytest.mark.asyncio
async def test_chunk_content_with_long_text(embedding_service):
    """Test content chunking with longer text."""
    # Create text that will definitely need chunking
    long_text = "This is a sentence. " * 50  # 50 sentences
    content = TextbookContent(
        content_id="test_content_456",
        title="Long Test Content",
        text=long_text,
        module="test_module",
        chapter="test_chapter",
        section="test_section",
    )

    chunks = embedding_service.chunk_content(content, max_chunk_size=50, overlap=10)

    assert isinstance(chunks, list)
    assert len(chunks) > 1  # Should be split into multiple chunks

    # Verify all chunks have correct metadata
    for chunk in chunks:
        assert isinstance(chunk, ContentChunk)
        assert chunk.content_id == "test_content_456"
        assert chunk.module == "test_module"
        assert chunk.chapter == "test_chapter"
        assert chunk.section == "test_section"
        assert len(chunk.text_chunk) > 0

    # Verify chunk indices are sequential
    chunk_indices = [c.chunk_index for c in chunks]
    assert chunk_indices == list(range(len(chunk_indices)))


@pytest.mark.asyncio
async def test_chunk_content_empty_text(embedding_service):
    """Test content chunking with empty text."""
    content = TextbookContent(
        content_id="test_content_empty",
        title="Empty Content",
        text="",
        module="test_module",
        chapter="test_chapter",
        section="test_section",
    )

    chunks = embedding_service.chunk_content(content)

    # Should return an empty list or list with one empty chunk
    assert isinstance(chunks, list)
    # If text is empty, no chunks should be created


@pytest.mark.asyncio
async def test_retrieve_similar_embeddings_success(embedding_service):
    """Test successful retrieval of similar embeddings."""
    query_embedding = [0.5, 0.6, 0.7]
    top_k = 3

    with patch.object(
        embedding_service.qdrant, "search_similar", new_callable=AsyncMock
    ) as mock_search:
        with patch.object(
            embedding_service.db, "get_embedding_by_id", new_callable=AsyncMock
        ) as mock_get_embedding:
            # Mock search results
            search_results = [
                type(
                    "obj",
                    (object,),
                    {
                        "embedding_id": "emb_1",
                        "content_id": "content_1",
                        "module": "module_1",
                        "chapter": "ch_1",
                        "section": "sec_1",
                        "relevance_score": 0.95,
                        "metadata": {},
                    },
                )(),
                type(
                    "obj",
                    (object,),
                    {
                        "embedding_id": "emb_2",
                        "content_id": "content_2",
                        "module": "module_2",
                        "chapter": "ch_2",
                        "section": "sec_2",
                        "relevance_score": 0.87,
                        "metadata": {},
                    },
                )(),
                type(
                    "obj",
                    (object,),
                    {
                        "embedding_id": "emb_3",
                        "content_id": "content_3",
                        "module": "module_3",
                        "chapter": "ch_3",
                        "section": "sec_3",
                        "relevance_score": 0.78,
                        "metadata": {},
                    },
                )(),
            ]
            mock_search.return_value = search_results

            # Mock database retrieval
            embedding_models = []
            for result in search_results:
                emb_model = EmbeddingModel(
                    embedding_id=result.embedding_id,
                    content_id=result.content_id,
                    embedding=[0.1, 0.2, 0.3],  # Mock embedding
                    model_name="embed-english-v3.0",
                    module=result.module,
                    chapter=result.chapter,
                    section=result.section,
                    metadata=result.metadata,
                )
                embedding_models.append(emb_model)

            mock_get_embedding.side_effect = embedding_models

            # Call the method
            result = await embedding_service.retrieve_similar_embeddings(
                query_embedding, top_k
            )

            # Verify the result
            assert isinstance(result, list)
            assert len(result) == 3  # Should match top_k
            for emb_model in result:
                assert isinstance(emb_model, EmbeddingModel)
                assert emb_model.embedding_id in ["emb_1", "emb_2", "emb_3"]


@pytest.mark.asyncio
async def test_process_and_store_content_success(embedding_service):
    """Test successful processing and storage of content."""
    content = TextbookContent(
        content_id="test_content_789",
        title="Process and Store Test",
        text="This content will be processed and stored with embeddings.",
        module="test_module",
        chapter="test_chapter",
        section="test_section",
    )

    with patch.object(
        embedding_service.db, "insert_content", new_callable=AsyncMock
    ) as mock_insert_content:
        with patch.object(
            embedding_service, "create_embeddings_for_content", new_callable=AsyncMock
        ) as mock_create_embeddings:
            mock_insert_content.return_value = "test_content_789"

            # Mock embedding creation
            embedding_models = [
                EmbeddingModel(
                    embedding_id="emb_1",
                    content_id="test_content_789",
                    embedding=[0.1, 0.2, 0.3],
                    model_name="embed-english-v3.0",
                    module="test_module",
                    chapter="test_chapter",
                    section="test_section",
                ),
                EmbeddingModel(
                    embedding_id="emb_2",
                    content_id="test_content_789",
                    embedding=[0.4, 0.5, 0.6],
                    model_name="embed-english-v3.0",
                    module="test_module",
                    chapter="test_chapter",
                    section="test_section",
                ),
            ]
            mock_create_embeddings.return_value = embedding_models

            # Call the method
            result = await embedding_service.process_and_store_content(content)

            # Verify the result
            assert isinstance(result, list)
            assert len(result) == 2  # Should match number of embeddings created
            assert result == ["emb_1", "emb_2"]

            # Verify method calls
            mock_insert_content.assert_called_once()
            mock_create_embeddings.assert_called_once_with(content)


@pytest.mark.asyncio
async def test_delete_embeddings_for_content_success(embedding_service):
    """Test successful deletion of embeddings for content."""
    content_id = "test_content_delete"

    with patch.object(
        embedding_service.db, "get_embedding_ids_by_content_id", new_callable=AsyncMock
    ) as mock_get_ids:
        with patch.object(
            embedding_service.qdrant, "delete_embedding", new_callable=AsyncMock
        ) as mock_qdrant_delete:
            with patch.object(
                embedding_service.db,
                "delete_embeddings_by_content_id",
                new_callable=AsyncMock,
            ) as mock_db_delete:
                mock_get_ids.return_value = ["emb_1", "emb_2", "emb_3"]

                # Call the method
                result = await embedding_service.delete_embeddings_for_content(
                    content_id
                )

                # Verify the result
                assert result is True

                # Verify method calls
                mock_get_ids.assert_called_once_with(content_id)
                assert (
                    mock_qdrant_delete.call_count == 3
                )  # Called for each embedding ID
                mock_db_delete.assert_called_once_with(content_id)


@pytest.mark.asyncio
async def test_embedding_error_handling(embedding_service):
    """Test error handling in embedding generation."""
    test_text = "This is a test sentence."

    with patch.object(
        embedding_service.cohere_service, "generate_embedding", new_callable=AsyncMock
    ) as mock_generate:
        mock_generate.side_effect = Exception("Cohere API error")

        with pytest.raises(Exception) as exc_info:
            await embedding_service.generate_embedding(test_text)

        assert "Cohere API error" in str(exc_info.value)


@pytest.mark.asyncio
async def test_embeddings_batch_error_handling(embedding_service):
    """Test error handling in batch embedding generation."""
    test_texts = ["Text 1", "Text 2"]

    with patch.object(
        embedding_service.cohere_service,
        "generate_embeddings_batch",
        new_callable=AsyncMock,
    ) as mock_generate_batch:
        mock_generate_batch.side_effect = Exception("Cohere batch API error")

        with pytest.raises(Exception) as exc_info:
            await embedding_service.generate_embeddings_batch(test_texts)

        assert "Cohere batch API error" in str(exc_info.value)


@pytest.mark.asyncio
async def test_content_chunking_edge_cases(embedding_service):
    """Test content chunking edge cases."""
    # Test with text shorter than chunk size
    short_content = TextbookContent(
        content_id="short_content",
        title="Short Content",
        text="Short text",
        module="test_module",
        chapter="test_chapter",
        section="test_section",
    )

    chunks = embedding_service.chunk_content(
        short_content, max_chunk_size=100, overlap=10
    )
    assert len(chunks) >= 1
    assert chunks[0].text_chunk == "Short text"

    # Test with single long word
    single_word_content = TextbookContent(
        content_id="single_word",
        title="Single Word",
        text="Supercalifragilisticexpialidocious",
        module="test_module",
        chapter="test_chapter",
        section="test_section",
    )

    chunks = embedding_service.chunk_content(
        single_word_content, max_chunk_size=10, overlap=2
    )
    assert len(chunks) >= 1
    # The long word might be split or kept as is depending on the implementation


if __name__ == "__main__":
    pytest.main([__file__])
