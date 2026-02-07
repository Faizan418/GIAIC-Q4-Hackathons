#!/usr/bin/env python3
"""Test script to verify the embedding integration is working properly."""

import asyncio
import logging

from backend.embeddings.cohere_embed import cohere_service
from backend.services.embedding_service import embedding_service
from backend.vectorstore.qdrant_client import qdrant_client

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def test_embedding_integration():
    """Test the complete embedding pipeline."""
    print("🔍 Testing Embedding Integration Pipeline")
    print("=" * 50)

    try:
        # Step 1: Test Cohere connection
        print("1. Testing Cohere connection...")
        test_text = "This is a test sentence for embedding."
        test_embedding = await cohere_service.generate_embedding(
            test_text, input_type="search_query"
        )
        print(f"   ✅ Cohere working - Embedding dimension: {len(test_embedding)}")

        # Step 2: Test Qdrant initialization
        print("2. Testing Qdrant initialization...")
        await qdrant_client.initialize()
        print("   ✅ Qdrant initialized successfully")

        # Step 3: Test storing an embedding
        print("3. Testing embedding storage...")
        import uuid

        test_id = str(uuid.uuid4())
        stored_id = await qdrant_client.store_embedding(
            embedding_id=test_id,
            embedding=test_embedding,
            content_id="test_content",
            module="test_module",
            chapter="test_chapter",
            section="test_section",
        )
        print(f"   ✅ Embedding stored with ID: {stored_id}")

        # Step 4: Test search
        print("4. Testing embedding search...")
        search_results = await qdrant_client.search_similar(
            query_vector=test_embedding, top_k=1
        )
        print(f"   ✅ Search working - Found {len(search_results)} results")

        # Step 5: Test full service integration
        print("5. Testing full service integration...")
        # Note: This assumes we have a test document to process
        # For now, we'll just test the individual components

        # Step 6: Test the embedding service
        print("6. Testing embedding service...")
        query_embedding = await embedding_service.generate_embedding(
            "test query for search", input_type="search_query"
        )
        print(
            f"   ✅ Embedding service working - Query embedding dimension: {len(query_embedding)}"
        )

        # Step 7: Test batch embedding
        print("7. Testing batch embedding...")
        texts = ["test document 1", "test document 2", "test document 3"]
        batch_embeddings = await embedding_service.generate_embeddings_batch(texts)
        print(
            f"   ✅ Batch embedding working - Generated {len(batch_embeddings)} embeddings"
        )

        print(
            "\n🎉 All integration tests passed! Embedding pipeline is working correctly."
        )

    except Exception as e:
        logger.error(f"❌ Integration test failed: {str(e)}", exc_info=True)
        return False

    return True


if __name__ == "__main__":
    success = asyncio.run(test_embedding_integration())
    if not success:
        exit(1)
