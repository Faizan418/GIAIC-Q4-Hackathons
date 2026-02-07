#!/usr/bin/env python3
"""Test script to verify Qdrant connection and embedding storage."""

import asyncio
import logging
from uuid import uuid4

from backend.config import settings
from backend.vectorstore.qdrant_client import qdrant_client

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def test_qdrant_connection():
    """Test Qdrant connection and basic operations."""
    print("🔍 Testing Qdrant Connection")
    print("=" * 50)
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Qdrant Collection: {settings.qdrant_collection}")
    print(f"Qdrant API Key: {'***' if settings.qdrant_api_key else 'None'}")
    print()

    try:
        # Test 1: Initialize Qdrant
        print("1. Initializing Qdrant...")
        await qdrant_client.initialize()
        print("   ✅ Qdrant initialized successfully")
        print()

        # Test 2: Check if collection exists
        print("2. Checking collection...")
        collections = await qdrant_client.client.get_collections()
        collection_names = [col.name for col in collections.collections]
        print(f"   Available collections: {collection_names}")

        if settings.qdrant_collection in collection_names:
            print(f"   ✅ Collection '{settings.qdrant_collection}' exists")
        else:
            print(
                f"   ⚠️  Collection '{settings.qdrant_collection}' does not exist yet"
            )
        print()

        # Test 3: Try to store a test embedding
        print("3. Testing embedding storage...")
        test_embedding_id = f"test_embedding_{uuid4().hex[:8]}"
        test_embedding = [
            0.1
        ] * 768  # 768-dimensional vector for multilingual-v2.0 model

        await qdrant_client.store_embedding(
            embedding_id=test_embedding_id,
            embedding=test_embedding,
            content_id="test_content",
            module="test_module",
            chapter="test_chapter",
            section="test_section",
        )
        print(f"   ✅ Test embedding stored with ID: {test_embedding_id}")
        print()

        # Test 4: Search for the stored embedding
        print("4. Testing embedding search...")
        search_results = await qdrant_client.search_similar(
            query_vector=test_embedding, top_k=1
        )
        print(f"   ✅ Search completed - Found {len(search_results)} results")

        if search_results:
            result = search_results[0]
            print(f"   - Found embedding ID: {result['embedding_id']}")
            print(f"   - Content ID: {result['content_id']}")
            print(f"   - Relevance score: {result['relevance_score']:.4f}")
        print()

        # Test 5: Try to retrieve the embedding
        print("5. Testing embedding retrieval...")
        retrieved = await qdrant_client.get_embedding(test_embedding_id)
        if retrieved:
            print(f"   ✅ Retrieved embedding with ID: {retrieved['id']}")
            print(f"   - Vector dimension: {len(retrieved['vector'])}")
        else:
            print(f"   ⚠️  Could not retrieve embedding with ID: {test_embedding_id}")
        print()

        print("🎉 Qdrant connection test completed successfully!")
        return True

    except Exception as e:
        logger.error(f"❌ Qdrant connection test failed: {str(e)}", exc_info=True)
        return False


if __name__ == "__main__":
    success = asyncio.run(test_qdrant_connection())
    if not success:
        exit(1)
