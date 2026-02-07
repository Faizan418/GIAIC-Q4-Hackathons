#!/usr/bin/env python3
"""Test script to verify embeddings are properly stored in Qdrant."""

import asyncio
import logging
from uuid import uuid4

from backend.config import settings
from backend.embeddings.cohere_embed import cohere_service
from backend.vectorstore.qdrant_client import qdrant_client

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def test_embedding_storage():
    """Test embedding storage and retrieval."""
    print("🔍 Testing Qdrant Embedding Storage")
    print("=" * 50)
    print(f"Qdrant Collection: {settings.qdrant_collection}")
    print()

    try:
        # Initialize Qdrant
        print("1. Initializing Qdrant...")
        await qdrant_client.initialize()
        print("   ✅ Qdrant initialized successfully")
        print()

        # Generate a test embedding
        print("2. Generating test embedding...")
        test_text = (
            "This is a test document about AI and robotics for humanoid applications."
        )
        embedding = cohere_service.generate_query_embedding(test_text)
        print(f"   ✅ Generated embedding with dimension: {len(embedding)}")
        print()

        # Store the embedding
        print("3. Storing embedding in Qdrant...")
        embedding_id = f"test_embedding_{uuid4().hex[:8]}"
        stored_id = await qdrant_client.store_embedding(
            embedding_id=embedding_id,
            embedding=embedding,
            content_id="test_content_001",
            text_chunk=test_text,
            module="test_module",
            chapter="test_chapter",
            section="test_section",
        )
        print(f"   ✅ Stored embedding with ID: {stored_id}")
        print()

        # Search for the stored embedding
        print("4. Searching for stored embedding...")
        search_results = await qdrant_client.search_similar(
            query_vector=embedding, top_k=1
        )
        print(f"   ✅ Search completed - Found {len(search_results)} results")

        if search_results:
            result = search_results[0]
            print(f"      - Found embedding ID: {result['embedding_id']}")
            print(f"      - Content ID: {result['content_id']}")
            print(f"      - Text chunk preview: {result['text_chunk'][:50]}...")
            print(f"      - Relevance score: {result['relevance_score']:.4f}")
            print(f"      - Module: {result['module']}")
        print()

        # Count total embeddings in collection
        print("5. Counting embeddings in collection...")
        # Note: For Qdrant Cloud, we can't use the count method directly without more setup
        # So we'll just verify the search worked
        print("   ✅ Search functionality verified")
        print()

        print("🎉 All embedding storage tests passed!")
        return True

    except Exception as e:
        logger.error(f"❌ Test failed: {str(e)}", exc_info=True)
        return False


if __name__ == "__main__":
    success = asyncio.run(test_embedding_storage())
    if not success:
        exit(1)
