#!/usr/bin/env python3
"""
Embedding Service for Qdrant Vector Database
This service handles the creation and storage of embeddings in Qdrant
for RAG functionality.
"""

import logging
import os
from typing import Any, Dict, List, Optional
from uuid import uuid4

try:
    import cohere
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    from qdrant_client.http.models import PointStruct
except ImportError as e:
    print(f"Required packages not found: {e}")
    print("Please install: pip install cohere qdrant-client")
    raise


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class EmbeddingService:
    def __init__(
        self,
        cohere_api_key: Optional[str] = None,
        qdrant_url: Optional[str] = None,
        qdrant_api_key: Optional[str] = None,
    ):
        """
        Initialize the embedding service with Cohere and Qdrant clients
        """
        self.cohere_api_key = cohere_api_key or os.getenv("COHERE_API_KEY")
        self.qdrant_url = qdrant_url or os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key = qdrant_api_key or os.getenv("QDRANT_API_KEY")

        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        # Initialize clients
        self.cohere_client = cohere.Client(self.cohere_api_key)
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url, api_key=self.qdrant_api_key, prefer_grpc=False
        )

        # Collection name for embeddings
        self.collection_name = "textbook_embeddings"

        # Create collection if it doesn't exist
        self._create_collection_if_not_exists()

    def _create_collection_if_not_exists(self):
        """
        Create Qdrant collection if it doesn't exist
        """
        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections()
            collection_exists = any(
                col.name == self.collection_name for col in collections.collections
            )

            if not collection_exists:
                # Create collection with appropriate vector size (Cohere embeddings are 1024-dim)
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1024,  # Cohere embedding dimension
                        distance=models.Distance.COSINE,
                    ),
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {e}")
            raise

    def create_embedding(self, text: str) -> List[float]:
        """
        Create a single embedding for the given text
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-english-v3.0",  # Using Cohere's latest embedding model
                input_type="search_document",  # Optimize for search
            )
            return response.embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            logger.error(f"Error generating embedding for text: {e}")
            raise

    def batch_create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for a batch of texts
        """
        if len(texts) > 96:  # Cohere API limit
            raise ValueError("Batch size cannot exceed 96 texts due to API limits")

        try:
            response = self.cohere_client.embed(
                texts=texts, model="embed-english-v3.0", input_type="search_document"
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            logger.error(f"Error generating batch embeddings: {e}")
            raise

    def store_embedding(
        self,
        text: str,
        metadata: Optional[Dict[str, Any]] = None,
        point_id: Optional[str] = None,
    ) -> str:
        """
        Create and store a single embedding in Qdrant
        """
        if point_id is None:
            point_id = str(uuid4())

        try:
            # Generate embedding
            embedding = self.create_embedding(text)

            # Prepare payload
            payload = {"text": text, "metadata": metadata or {}}

            # Create point
            point = PointStruct(id=point_id, vector=embedding, payload=payload)

            # Store in Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name, points=[point]
            )

            logger.info(f"Stored embedding with ID: {point_id}")
            return point_id

        except Exception as e:
            logger.error(f"Error storing embedding: {e}")
            raise

    def batch_store_embeddings(
        self, texts: List[str], metadatas: Optional[List[Dict[str, Any]]] = None
    ) -> List[str]:
        """
        Create and store multiple embeddings in Qdrant
        """
        if metadatas is None:
            metadatas = [{} for _ in texts]

        if len(texts) != len(metadatas):
            raise ValueError("Texts and metadatas must have the same length")

        # Process in batches to respect API limits
        batch_size = min(96, len(texts))  # Cohere API limit
        all_point_ids = []

        for i in range(0, len(texts), batch_size):
            batch_texts = texts[i : i + batch_size]
            batch_metadatas = metadatas[i : i + batch_size]

            try:
                # Generate embeddings for the batch
                embeddings = self.batch_create_embeddings(batch_texts)

                # Prepare points
                points = []
                for j, (text, embedding, metadata) in enumerate(
                    zip(batch_texts, embeddings, batch_metadatas)
                ):
                    point_id = str(uuid4())
                    point = PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={"text": text, "metadata": metadata},
                    )
                    points.append(point)
                    all_point_ids.append(point_id)

                # Store batch in Qdrant
                self.qdrant_client.upsert(
                    collection_name=self.collection_name, points=points
                )

                logger.info(f"Stored batch of {len(points)} embeddings")

            except Exception as e:
                logger.error(f"Error storing batch of embeddings: {e}")
                raise

        return all_point_ids

    def search_similar(self, query: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in Qdrant
        """
        try:
            # Generate embedding for the query
            query_embedding = self.create_embedding(query)

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
            )

            # Format results
            results = []
            for result in search_results:
                results.append(
                    {
                        "id": result.id,
                        "text": result.payload.get("text", ""),
                        "metadata": result.payload.get("metadata", {}),
                        "score": result.score,
                    }
                )

            return results

        except Exception as e:
            logger.error(f"Error searching for similar embeddings: {e}")
            raise

    def get_embedding_count(self) -> int:
        """
        Get the total count of embeddings in the collection
        """
        try:
            count = self.qdrant_client.count(collection_name=self.collection_name)
            return count.count
        except Exception as e:
            logger.error(f"Error getting embedding count: {e}")
            return 0

    def delete_embedding(self, point_id: str) -> bool:
        """
        Delete a specific embedding by ID
        """
        try:
            self.qdrant_client.delete(
                collection_name=self.collection_name, points_selector=[point_id]
            )
            logger.info(f"Deleted embedding with ID: {point_id}")
            return True
        except Exception as e:
            logger.error(f"Error deleting embedding: {e}")
            return False


# Example usage and testing
if __name__ == "__main__":
    # Initialize the service
    service = EmbeddingService()

    # Example: Store a single embedding
    text = "Physical AI and Humanoid Robotics represent the cutting edge of artificial intelligence research, combining machine learning with embodied systems."
    metadata = {
        "source": "textbook",
        "category": "introduction",
        "timestamp": "2025-12-17",
    }

    print("Starting embedding and storing in Qdrant...")

    # Store a single embedding
    point_id = service.store_embedding(text, metadata)
    print(f"Stored embedding with ID: {point_id}")

    # Store multiple embeddings
    texts = [
        "Deep reinforcement learning is used to train humanoid robots for complex tasks.",
        "Sim-to-real transfer is crucial for deploying AI models in physical environments.",
        "Embodied cognition connects perception and action in intelligent systems.",
    ]

    metadatas = [
        {"source": "chapter_1", "page": 15},
        {"source": "chapter_2", "page": 42},
        {"source": "chapter_3", "page": 78},
    ]

    point_ids = service.batch_store_embeddings(texts, metadatas)
    print(f"Stored {len(point_ids)} embeddings in batch: {point_ids}")

    # Search for similar content
    query = "How is AI used in robotics?"
    results = service.search_similar(query, limit=3)
    print(f"Search results for '{query}':")
    for i, result in enumerate(results, 1):
        print(f"  {i}. Score: {result['score']:.3f}, Text: {result['text'][:100]}...")

    # Get total count
    total_count = service.get_embedding_count()
    print(f"Total embeddings in database: {total_count}")

    print("Embedding service test completed successfully!")
