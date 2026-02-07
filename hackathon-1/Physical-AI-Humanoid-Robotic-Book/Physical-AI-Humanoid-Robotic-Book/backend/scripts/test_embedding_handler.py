#!/usr/bin/env python3
"""
Test script for Text Embedding Handler
"""

import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import the handler
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

from scripts.text_embedding_handler import TextEmbeddingHandler


def test_text_embedding_handler():
    """
    Test the TextEmbeddingHandler functionality
    """
    print("🧪 Testing Text Embedding Handler...")

    # Check if required environment variables are set
    cohere_key = os.getenv("COHERE_API_KEY")
    if not cohere_key:
        print("⚠️  WARNING: COHERE_API_KEY environment variable not set")
        print("   Please set COHERE_API_KEY to run full tests")
        return

    try:
        # Initialize the handler
        print("✅ Initializing TextEmbeddingHandler...")
        handler = TextEmbeddingHandler(collection_name="test_embeddings")

        # Test 1: Text chunking
        print("\n📝 Testing text chunking...")
        test_text = (
            "This is a sample text for testing. " * 50
        )  # Make it long enough to chunk
        chunks = handler.chunk_text(test_text, chunk_size=50, overlap=10)
        print(f"   Original text length: {len(test_text)}")
        print(f"   Number of chunks: {len(chunks)}")
        print(f"   First chunk: '{chunks[0][:30]}...'")
        print(f"   Last chunk: '{chunks[-1][:30]}...'")
        assert len(chunks) > 1, "Text should be chunked into multiple pieces"
        print("   ✅ Text chunking test passed")

        # Test 2: Embedding generation
        print("\n🌐 Testing embedding generation...")
        sample_texts = ["Hello world", "How are you?", "This is a test"]
        embeddings = handler.embed_texts(sample_texts, input_type="search_document")
        print(f"   Generated {len(embeddings)} embeddings")
        print(f"   Embedding dimension: {len(embeddings[0])}")
        assert len(embeddings) == len(
            sample_texts
        ), "Should generate one embedding per text"
        assert (
            len(embeddings[0]) == 1024
        ), "Cohere embeddings should be 1024-dimensional"
        print("   ✅ Embedding generation test passed")

        # Test 3: Single embedding
        print("\n🔹 Testing single embedding...")
        single_embedding = handler.embed_single_text(
            "Single test", input_type="search_query"
        )
        print(f"   Single embedding dimension: {len(single_embedding)}")
        assert (
            len(single_embedding) == 1024
        ), "Single embedding should be 1024-dimensional"
        print("   ✅ Single embedding test passed")

        # Test 4: Upsert functionality
        print("\n💾 Testing upsert functionality...")
        test_texts = ["Test document 1", "Test document 2", "Test document 3"]
        test_metadatas = [
            {"source": "test", "id": "1"},
            {"source": "test", "id": "2"},
            {"source": "test", "id": "3"},
        ]

        # Upsert embeddings
        ids = handler.upsert_embeddings(
            texts=test_texts, metadatas=test_metadatas, input_type="search_document"
        )
        print(f"   Upserted {len(ids)} embeddings with IDs: {ids}")
        assert len(ids) == len(test_texts), "Should have an ID for each text"
        print("   ✅ Upsert functionality test passed")

        # Test 5: Search functionality
        print("\n🔍 Testing search functionality...")
        search_results = handler.search_similar("test", limit=2)
        print(f"   Found {len(search_results)} search results")
        if search_results:
            print(f"   First result score: {search_results[0]['score']:.3f}")
            print(f"   First result text: {search_results[0]['text'][:50]}...")
        print("   ✅ Search functionality test passed")

        # Test 6: Count functionality
        print("\n📊 Testing count functionality...")
        count = handler.get_embedding_count()
        print(f"   Total embeddings in collection: {count}")
        assert count >= len(test_texts), "Count should include our test embeddings"
        print("   ✅ Count functionality test passed")

        # Test 7: Single embedding upsert
        print("\n🔹 Testing single embedding upsert...")
        single_id = handler.upsert_single_embedding(
            text="Single upsert test",
            metadata={"source": "single_test"},
            input_type="search_document",
        )
        print(f"   Upserted single embedding with ID: {single_id}")
        assert single_id is not None, "Should return a valid ID"
        print("   ✅ Single embedding upsert test passed")

        print(
            f"\n🎉 All tests passed! Total embeddings in test collection: {handler.get_embedding_count()}"
        )

        # Clean up test data
        print("\n🧹 Cleaning up test data...")
        all_ids = ids + [single_id]
        success = handler.delete_embeddings_by_ids(all_ids)
        if success:
            print(f"   Deleted {len(all_ids)} test embeddings")
        else:
            print("   ⚠️  Failed to delete test embeddings")

        print("\n✅ All tests completed successfully!")

    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback

        traceback.print_exc()
        return False

    return True


if __name__ == "__main__":
    success = test_text_embedding_handler()
    if not success:
        sys.exit(1)
