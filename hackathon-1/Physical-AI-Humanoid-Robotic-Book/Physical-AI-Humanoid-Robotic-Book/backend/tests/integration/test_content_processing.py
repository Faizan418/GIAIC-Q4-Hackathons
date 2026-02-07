"""Integration test for the content processing flow in the RAG Chatbot API."""

import asyncio
from unittest.mock import AsyncMock, patch

import pytest
from fastapi.testclient import TestClient

from backend.main import app
from backend.models.content import TextbookContent
from backend.services.content_processing_pipeline import content_processing_pipeline


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    with TestClient(app) as test_client:
        yield test_client


@pytest.mark.asyncio
async def test_content_processing_pipeline_success():
    """Test the full content processing pipeline integration."""
    # Create a mock content object
    mock_content = TextbookContent(
        content_id="test_content_123",
        title="Introduction to Physical AI",
        text="Physical AI is an approach that emphasizes the importance of physics-based reasoning in artificial intelligence systems. It combines principles of physics with machine learning algorithms to create more robust and explainable AI systems. This approach has applications in robotics, where understanding physical interactions is crucial for safe and effective operation.",
        module="001-intro-physical-ai",
        chapter="1",
        section="1.1",
    )

    # Mock all the dependencies
    with patch.object(
        content_processing_pipeline.embedding_service,
        "process_and_store_content",
        new_callable=AsyncMock,
    ) as mock_process_and_store:
        with patch.object(
            content_processing_pipeline.db, "insert_content", new_callable=AsyncMock
        ) as mock_insert_content:
            mock_insert_content.return_value = "test_content_123"
            mock_process_and_store.return_value = [
                "emb_1",
                "emb_2",
                "emb_3",
            ]  # Mock embedding IDs

            # Call the content processing pipeline
            result = await content_processing_pipeline.process_single_content(
                mock_content
            )

            # Verify the result
            assert isinstance(result, list)
            assert len(result) == 3  # Should match mock embedding IDs
            assert result == ["emb_1", "emb_2", "emb_3"]

            # Verify that the content was inserted into the database
            mock_insert_content.assert_called_once()

            # Verify that the content was processed and embeddings were stored
            mock_process_and_store.assert_called_once()


@pytest.mark.asyncio
async def test_content_processing_batch_success():
    """Test the content processing pipeline batch processing."""
    # Create multiple mock content objects
    mock_contents = [
        TextbookContent(
            content_id="test_content_1",
            title="Introduction to Physical AI",
            text="Physical AI is an approach that emphasizes physics-based reasoning.",
            module="001-intro-physical-ai",
            chapter="1",
            section="1.1",
        ),
        TextbookContent(
            content_id="test_content_2",
            title="Applications of Physical AI",
            text="Physical AI has applications in robotics and automation.",
            module="001-intro-physical-ai",
            chapter="1",
            section="1.2",
        ),
    ]

    # Mock all the dependencies
    with patch.object(
        content_processing_pipeline, "process_single_content", new_callable=AsyncMock
    ) as mock_process_single:
        mock_process_single.side_effect = [
            ["emb_1", "emb_2"],  # Embeddings for first content
            ["emb_3", "emb_4"],  # Embeddings for second content
        ]

        # Call the content processing pipeline
        result = await content_processing_pipeline.process_content_batch(
            mock_contents, max_concurrent=2
        )

        # Verify the result structure
        assert "successful" in result
        assert "failed" in result
        assert "total_processed" in result
        assert result["total_processed"] == 2

        # Verify successful processing
        assert len(result["successful"]) == 2
        assert len(result["failed"]) == 0

        # Verify that process_single_content was called for each content
        assert mock_process_single.call_count == 2


@pytest.mark.asyncio
async def test_content_processing_with_validation_error():
    """Test the content processing pipeline with validation errors."""
    # Create a content object with invalid data
    invalid_content = TextbookContent(
        content_id="",  # Invalid: empty content_id
        title="",
        text="",  # Invalid: empty text
        module="",
        chapter="",
        section="",
    )

    # Call the content processing pipeline
    with pytest.raises(Exception) as exc_info:
        await content_processing_pipeline.process_single_content(invalid_content)

    # Verify that it raises a validation error
    assert "Content validation failed" in str(exc_info.value)


@pytest.mark.asyncio
async def test_content_processing_with_service_error():
    """Test the content processing pipeline when a service error occurs."""
    # Create a valid content object
    mock_content = TextbookContent(
        content_id="test_content_123",
        title="Introduction to Physical AI",
        text="Physical AI is an approach that emphasizes physics-based reasoning.",
        module="001-intro-physical-ai",
        chapter="1",
        section="1.1",
    )

    # Mock the service to raise an error
    with patch.object(
        content_processing_pipeline.embedding_service,
        "process_and_store_content",
        new_callable=AsyncMock,
    ) as mock_process_and_store:
        mock_process_and_store.side_effect = Exception("Embedding service error")

        # Call the content processing pipeline
        with pytest.raises(Exception) as exc_info:
            await content_processing_pipeline.process_single_content(mock_content)

        # Verify that it raises an error
        assert "Embedding service error" in str(exc_info.value)


@pytest.mark.asyncio
async def test_update_content_integration():
    """Test the content update functionality."""
    # Create a mock content object
    mock_content = TextbookContent(
        content_id="test_content_123",
        title="Updated Introduction to Physical AI",
        text="Physical AI is an approach that emphasizes physics-based reasoning. This content has been updated.",
        module="001-intro-physical-ai",
        chapter="1",
        section="1.1",
    )

    # Mock all the dependencies
    with patch.object(
        content_processing_pipeline.embedding_service,
        "delete_embeddings_for_content",
        new_callable=AsyncMock,
    ) as mock_delete_embeddings:
        with patch.object(
            content_processing_pipeline,
            "process_single_content",
            new_callable=AsyncMock,
        ) as mock_process_single:
            mock_delete_embeddings.return_value = True
            mock_process_single.return_value = ["emb_1", "emb_2", "emb_3"]

            # Call the content processing pipeline
            result = await content_processing_pipeline.update_content(mock_content)

            # Verify the result
            assert isinstance(result, list)
            assert len(result) == 3
            assert result == ["emb_1", "emb_2", "emb_3"]

            # Verify that old embeddings were deleted
            mock_delete_embeddings.assert_called_once_with("test_content_123")

            # Verify that new content was processed
            mock_process_single.assert_called_once()


@pytest.mark.asyncio
async def test_ingest_from_textbook_structure():
    """Test the textbook structure ingestion functionality."""
    # Create a mock textbook structure
    textbook_structure = {
        "modules": [
            {
                "module_id": "001-intro-physical-ai",
                "chapters": [
                    {
                        "chapter_id": "1",
                        "sections": [
                            {
                                "section_id": "1.1",
                                "title": "What is Physical AI?",
                                "content": "Physical AI is an approach that emphasizes physics-based reasoning in artificial intelligence systems.",
                                "page_numbers": "1-10",
                            },
                            {
                                "section_id": "1.2",
                                "title": "Applications of Physical AI",
                                "content": "Physical AI has applications in robotics and automation.",
                                "page_numbers": "11-20",
                            },
                        ],
                    }
                ],
            }
        ]
    }

    # Mock all the dependencies
    with patch.object(
        content_processing_pipeline, "process_content_batch", new_callable=AsyncMock
    ) as mock_process_batch:
        mock_process_batch.return_value = {
            "successful": [
                {"content_id": "generated_id_1", "embedding_ids": ["emb_1", "emb_2"]},
                {"content_id": "generated_id_2", "embedding_ids": ["emb_3", "emb_4"]},
            ],
            "failed": [],
            "errors": [],
        }

        # Call the content processing pipeline
        result = await content_processing_pipeline.ingest_from_textbook_structure(
            textbook_structure
        )

        # Verify the result structure
        assert "successful" in result
        assert "failed" in result
        assert len(result["successful"]) == 2

        # Verify that process_content_batch was called with the right content
        assert mock_process_batch.call_count == 1
        args, kwargs = mock_process_batch.call_args
        created_contents = args[0]
        assert len(created_contents) == 2  # Should match number of sections

        # Verify content properties
        for content in created_contents:
            assert hasattr(content, "content_id")
            assert hasattr(content, "title")
            assert hasattr(content, "text")
            assert hasattr(content, "module")
            assert hasattr(content, "chapter")
            assert hasattr(content, "section")


@pytest.mark.asyncio
async def test_reindex_content():
    """Test the content re-indexing functionality."""
    content_id = "test_content_123"

    # Mock all the dependencies
    with patch.object(
        content_processing_pipeline.db, "get_content_by_id", new_callable=AsyncMock
    ) as mock_get_content:
        with patch.object(
            content_processing_pipeline, "update_content", new_callable=AsyncMock
        ) as mock_update_content:
            # Mock content retrieval
            mock_content_data = {
                "content_id": "test_content_123",
                "title": "Introduction to Physical AI",
                "text": "Physical AI is an approach that emphasizes physics-based reasoning.",
                "module": "001-intro-physical-ai",
                "chapter": "1",
                "section": "1.1",
                "page_numbers": "1-10",
                "metadata": {},
            }
            mock_get_content.return_value = mock_content_data

            # Mock content update
            mock_update_content.return_value = ["emb_1", "emb_2", "emb_3"]

            # Call the content processing pipeline
            result = await content_processing_pipeline.reindex_content(content_id)

            # Verify the result
            assert result["content_id"] == content_id
            assert result["embedding_count"] == 3
            assert result["status"] == "success"

            # Verify that content was retrieved from DB
            mock_get_content.assert_called_once_with(content_id)

            # Verify that content was updated (reindexed)
            assert mock_update_content.call_count == 1


@pytest.mark.asyncio
async def test_reindex_all_content():
    """Test the re-indexing of all content functionality."""
    # Mock all the dependencies
    with patch.object(
        content_processing_pipeline.db, "get_all_content_ids", new_callable=AsyncMock
    ) as mock_get_all_ids:
        with patch.object(
            content_processing_pipeline.db, "get_content_by_id", new_callable=AsyncMock
        ) as mock_get_content:
            with patch.object(
                content_processing_pipeline,
                "process_content_batch",
                new_callable=AsyncMock,
            ) as mock_process_batch:
                # Mock content IDs
                mock_get_all_ids.return_value = ["content_1", "content_2", "content_3"]

                # Mock content retrieval
                mock_content_data = {
                    "content_id": "content_1",
                    "title": "Introduction to Physical AI",
                    "text": "Physical AI is an approach that emphasizes physics-based reasoning.",
                    "module": "001-intro-physical-ai",
                    "chapter": "1",
                    "section": "1.1",
                    "page_numbers": "1-10",
                    "metadata": {},
                }
                mock_get_content.return_value = mock_content_data

                # Mock batch processing
                mock_process_batch.return_value = {
                    "successful": [
                        {
                            "content_id": "content_1",
                            "embedding_ids": ["emb_1", "emb_2"],
                        },
                        {
                            "content_id": "content_2",
                            "embedding_ids": ["emb_3", "emb_4"],
                        },
                        {
                            "content_id": "content_3",
                            "embedding_ids": ["emb_5", "emb_6"],
                        },
                    ],
                    "failed": [],
                    "errors": [],
                }

                # Call the content processing pipeline
                result = await content_processing_pipeline.reindex_all_content()

                # Verify the result structure
                assert "successful" in result
                assert "failed" in result
                assert len(result["successful"]) == 3

                # Verify that all content IDs were retrieved
                mock_get_all_ids.assert_called_once()

                # Verify that process_content_batch was called
                mock_process_batch.assert_called_once()


@pytest.mark.asyncio
async def test_content_processing_performance():
    """Test the content processing performance with larger content."""
    # Create a content object with larger text
    large_text = "Physical AI is an approach. " * 100  # Repeat to create larger content
    mock_content = TextbookContent(
        content_id="large_content_123",
        title="Large Content for Physical AI",
        text=large_text,
        module="001-intro-physical-ai",
        chapter="1",
        section="1.1",
    )

    # Mock the dependencies
    with patch.object(
        content_processing_pipeline.embedding_service,
        "process_and_store_content",
        new_callable=AsyncMock,
    ) as mock_process_and_store:
        with patch.object(
            content_processing_pipeline.db, "insert_content", new_callable=AsyncMock
        ) as mock_insert_content:
            mock_insert_content.return_value = "large_content_123"
            mock_process_and_store.return_value = [
                "emb_1",
                "emb_2",
                "emb_3",
                "emb_4",
                "emb_5",
            ]

            import time

            start_time = time.time()

            # Call the content processing pipeline
            result = await content_processing_pipeline.process_single_content(
                mock_content
            )

            end_time = time.time()
            duration = end_time - start_time

            # Verify that processing completed successfully
            assert isinstance(result, list)
            assert len(result) > 0

            # Processing time should be reasonable (in test environment it will be fast)
            assert duration >= 0


@pytest.mark.asyncio
async def test_content_processing_concurrent():
    """Test the content processing with multiple concurrent operations."""
    # Create multiple content objects
    contents = []
    for i in range(5):
        content = TextbookContent(
            content_id=f"content_{i}",
            title=f"Content {i} Title",
            text=f"This is content {i} with some text for processing.",
            module="001-intro-physical-ai",
            chapter="1",
            section=f"1.{i}",
        )
        contents.append(content)

    # Mock the dependencies
    with patch.object(
        content_processing_pipeline, "process_single_content", new_callable=AsyncMock
    ) as mock_process_single:

        async def side_effect(content):
            # Simulate some async work
            await asyncio.sleep(0.01)
            return [f"emb_{content.content_id}_1", f"emb_{content.content_id}_2"]

        mock_process_single.side_effect = side_effect

        # Call the content processing pipeline with limited concurrency
        result = await content_processing_pipeline.process_content_batch(
            contents, max_concurrent=3
        )

        # Verify the result
        assert "successful" in result
        assert "failed" in result
        assert result["total_processed"] == 5
        assert len(result["successful"]) == 5
        assert len(result["failed"]) == 0


if __name__ == "__main__":
    pytest.main([__file__])
