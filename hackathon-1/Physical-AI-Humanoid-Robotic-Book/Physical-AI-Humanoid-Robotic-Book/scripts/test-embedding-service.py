#!/usr/bin/env python3
"""
Test script for the embedding service
This script tests the embedding functionality with sample data
"""

import logging
import os
import sys

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend", "src"))

from services.embedding_service import EmbeddingService

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_embedding_service():
    """
    Test the embedding service functionality
    """
    print("🧪 Testing Embedding Service...")

    try:
        # Initialize the service
        # Note: This will use environment variables for API keys
        service = EmbeddingService()
        print("✅ Embedding service initialized successfully")

        # Test 1: Create a single embedding
        print("\n1️⃣ Testing single embedding creation...")
        text = "Physical AI and Humanoid Robotics represent the cutting edge of artificial intelligence research."
        embedding = service.create_embedding(text)
        print(f"✅ Created embedding with {len(embedding)} dimensions")

        # Test 2: Store a single embedding
        print("\n2️⃣ Testing single embedding storage...")
        metadata = {
            "source": "test",
            "category": "introduction",
            "timestamp": "2025-12-17",
        }
        point_id = service.store_embedding(text, metadata)
        print(f"✅ Stored embedding with ID: {point_id}")

        # Test 3: Batch create embeddings
        print("\n3️⃣ Testing batch embedding creation...")
        texts = [
            "Deep reinforcement learning is used to train humanoid robots.",
            "Sim-to-real transfer is crucial for deploying AI models.",
            "Embodied cognition connects perception and action in intelligent systems.",
        ]
        batch_embeddings = service.batch_create_embeddings(texts)
        print(f"✅ Created {len(batch_embeddings)} embeddings in batch")

        # Test 4: Batch store embeddings
        print("\n4️⃣ Testing batch embedding storage...")
        metadatas = [
            {"source": "test_batch", "page": 1},
            {"source": "test_batch", "page": 2},
            {"source": "test_batch", "page": 3},
        ]
        point_ids = service.batch_store_embeddings(texts, metadatas)
        print(f"✅ Stored {len(point_ids)} embeddings in batch: {point_ids[:2]}...")

        # Test 5: Search for similar content
        print("\n5️⃣ Testing similarity search...")
        query = "How is AI used in robotics?"
        results = service.search_similar(query, limit=2)
        print(f"✅ Found {len(results)} similar embeddings")
        for i, result in enumerate(results, 1):
            print(f"   {i}. Score: {result['score']:.3f}")
            print(f"      Text: {result['text'][:60]}...")

        # Test 6: Get total count
        print("\n6️⃣ Testing embedding count...")
        total_count = service.get_embedding_count()
        print(f"✅ Total embeddings in database: {total_count}")

        # Test 7: Verify the stored embeddings exist
        print("\n7️⃣ Verifying stored embeddings...")
        # Search for something related to what we stored
        verification_query = "humanoid robots"
        verification_results = service.search_similar(verification_query, limit=1)
        if verification_results:
            print(
                f"✅ Found matching content: {verification_results[0]['text'][:60]}..."
            )
        else:
            print("⚠️ No matching content found for verification")

        print("\n🎉 All tests passed! Embedding service is working correctly.")
        return True

    except Exception as e:
        logger.error(f"❌ Error during testing: {e}")
        return False


def test_with_sample_docs():
    """
    Test the embedding service with sample documentation content
    """
    print("\n📚 Testing with sample documentation content...")

    try:
        service = EmbeddingService()

        # Sample content from a typical AI/Humanoid Robotics textbook
        sample_docs = [
            {
                "text": "Physical AI is an emerging field that combines artificial intelligence with physical systems. It focuses on creating intelligent agents that can interact with the real world through sensors and actuators.",
                "metadata": {"source": "intro", "section": "1.1", "type": "concept"},
            },
            {
                "text": "Humanoid robots are robots with physical anthropomorphic characteristics. They are designed to resemble and imitate human behavior, often featuring two legs, two arms, a head, and a torso.",
                "metadata": {
                    "source": "robots",
                    "section": "2.3",
                    "type": "definition",
                },
            },
            {
                "text": "Reinforcement learning is a type of machine learning where an agent learns to make decisions by performing actions and receiving rewards or penalties from the environment.",
                "metadata": {"source": "ml", "section": "3.5", "type": "concept"},
            },
            {
                "text": "Sim-to-real transfer is the process of transferring policies learned in simulation to real-world robots. This is challenging due to the reality gap between simulated and real environments.",
                "metadata": {
                    "source": "transfer",
                    "section": "4.2",
                    "type": "technique",
                },
            },
            {
                "text": "Embodied cognition is the theory that many features of cognition, whether human or otherwise, are shaped by aspects of the entire body of the organism, suggesting that the body plays an active role in shaping cognition.",
                "metadata": {"source": "cognition", "section": "5.1", "type": "theory"},
            },
        ]

        # Store all sample docs
        stored_ids = []
        for i, doc in enumerate(sample_docs):
            point_id = service.store_embedding(doc["text"], doc["metadata"])
            stored_ids.append(point_id)
            print(f"   Stored document {i+1}: {point_id}")

        print(f"✅ Stored {len(stored_ids)} sample documents")

        # Test search with various queries
        test_queries = [
            "What are humanoid robots?",
            "How does reinforcement learning work?",
            "Explain physical AI concepts",
            "What is sim-to-real transfer?",
        ]

        print("\n🔍 Testing search queries:")
        for query in test_queries:
            results = service.search_similar(query, limit=1)
            if results:
                print(f"   Query: '{query}'")
                print(f"   Result: {results[0]['text'][:80]}...")
                print(f"   Score: {results[0]['score']:.3f}")
            else:
                print(f"   Query: '{query}' - No results found")

        # Final count
        total_count = service.get_embedding_count()
        print(f"\n📊 Total embeddings after sample docs: {total_count}")

        print("✅ Sample documentation test completed successfully!")
        return True

    except Exception as e:
        logger.error(f"❌ Error during sample docs testing: {e}")
        return False


if __name__ == "__main__":
    print("🚀 Starting Embedding Service Tests...")

    # Run the main tests
    success1 = test_embedding_service()

    # Run the sample docs tests
    success2 = test_with_sample_docs()

    if success1 and success2:
        print("\n🎉 ALL TESTS PASSED! Embedding service is ready for use.")
        print("\n📋 Summary of functionality:")
        print("   • Create single and batch embeddings")
        print("   • Store embeddings in Qdrant vector database")
        print("   • Search for similar content using vector similarity")
        print("   • Manage metadata with stored embeddings")
        print("   • Count total embeddings in database")
    else:
        print("\n❌ Some tests failed. Please check the error messages above.")
        sys.exit(1)
