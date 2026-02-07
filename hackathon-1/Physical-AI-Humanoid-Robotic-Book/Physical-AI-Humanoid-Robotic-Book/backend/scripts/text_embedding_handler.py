#!/usr/bin/env python3
"""
Text Embedding Handler using Cohere SDK and Qdrant

This script handles text embedding using Cohere's embed-english-v3.0 model with:
- Text chunking with overlap
- Proper input_type for search_document (for storage) and search_query (for questions)
- Upsert logic for Qdrant collection
- Error handling and environment variable management
"""

import logging
import os
from typing import Any, Dict, List, Optional
from uuid import uuid4

import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TextEmbeddingHandler:
    """
    Handles text embedding using Cohere SDK and storage in Qdrant.
    """

    def __init__(
        self,
        cohere_api_key: Optional[str] = None,
        qdrant_url: Optional[str] = None,
        qdrant_api_key: Optional[str] = None,
        collection_name: str = "book_embeddings",
    ):
        """
        Initialize the text embedding handler with Cohere and Qdrant clients.

        Args:
            cohere_api_key: Cohere API key. If None, will use COHERE_API_KEY env var
            qdrant_url: Qdrant URL. If None, will use QDRANT_URL env var
            qdrant_api_key: Qdrant API key. If None, will use QDRANT_API_KEY env var
            collection_name: Name of the Qdrant collection to use
        """
        self.cohere_api_key = cohere_api_key or os.getenv("COHERE_API_KEY")
        self.qdrant_url = qdrant_url or os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key = qdrant_api_key or os.getenv("QDRANT_API_KEY")
        self.collection_name = collection_name

        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        # Initialize Cohere client (using V2 as requested)
        self.cohere_client = cohere.Client(api_key=self.cohere_api_key)

        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url, api_key=self.qdrant_api_key, prefer_grpc=False
        )

        # Create collection if it doesn't exist
        self._create_collection_if_not_exists()

        logger.info("TextEmbeddingHandler initialized successfully")

    def _create_collection_if_not_exists(self):
        """
        Create Qdrant collection if it doesn't exist.
        """
        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections()
            collection_exists = any(
                col.name == self.collection_name for col in collections.collections
            )

            if not collection_exists:
                # Create collection with appropriate vector size (Cohere's embed-english-v3.0 uses 1024 dimensions)
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1024,  # Cohere's embedding dimension for embed-english-v3.0
                        distance=models.Distance.COSINE,
                    ),
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {e}")
            raise

    def chunk_text(
        self, text: str, chunk_size: int = 1000, overlap: int = 100
    ) -> List[str]:
        """
        Split text into overlapping chunks to preserve context.

        Args:
            text: The text to chunk
            chunk_size: Size of each chunk
            overlap: Number of characters to overlap between chunks

        Returns:
            List of text chunks
        """
        if len(text) <= chunk_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size
            chunk = text[start:end]
            chunks.append(chunk)

            # Move start forward by chunk_size minus overlap
            start = end - overlap

            # Ensure we don't get stuck in an infinite loop
            if start >= len(text) or len(chunk) < chunk_size:
                break

        return chunks

    def embed_texts(
        self, texts: List[str], input_type: str = "search_document"
    ) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere.

        Args:
            texts: List of texts to embed
            input_type: Type of input for embedding model. Use "search_document" for storage,
                       "search_query" for user questions, "classification", or "clustering"

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        if not texts:
            return []

        # Cohere has a limit on batch size, so we need to process in chunks if needed
        max_batch_size = 96  # Cohere's API limit

        all_embeddings = []

        for i in range(0, len(texts), max_batch_size):
            batch = texts[i : i + max_batch_size]

            try:
                response = self.cohere_client.embed(
                    texts=batch,
                    model="embed-english-v3.0",  # Using the latest model as requested
                    input_type=input_type,
                )

                # Extract embeddings from response
                batch_embeddings = [embedding for embedding in response.embeddings]
                all_embeddings.extend(batch_embeddings)

            except Exception as e:
                logger.error(f"Error generating embeddings for batch: {e}")
                raise

        return all_embeddings

    def embed_single_text(
        self, text: str, input_type: str = "search_document"
    ) -> List[float]:
        """
        Generate embedding for a single text using Cohere.

        Args:
            text: Text to embed
            input_type: Type of input for embedding model

        Returns:
            Embedding as a list of floats
        """
        return self.embed_texts([text], input_type)[0]

    def upsert_embeddings(
        self,
        texts: List[str],
        metadatas: Optional[List[Dict[str, Any]]] = None,
        ids: Optional[List[str]] = None,
        input_type: str = "search_document",
    ) -> List[str]:
        """
        Upsert text embeddings into Qdrant collection.

        Args:
            texts: List of texts to embed and store
            metadatas: Optional list of metadata dictionaries for each text
            ids: Optional list of IDs for the embeddings. If None, will generate UUIDs
            input_type: Type of input for embedding model

        Returns:
            List of IDs of the stored embeddings
        """
        if not texts:
            return []

        if metadatas is None:
            metadatas = [{} for _ in texts]

        if ids is None:
            ids = [str(uuid4()) for _ in texts]

        if len(texts) != len(metadatas):
            raise ValueError("texts and metadatas must have the same length")

        if len(texts) != len(ids):
            raise ValueError("texts and ids must have the same length")

        try:
            # Generate embeddings
            embeddings = self.embed_texts(texts, input_type)

            # Prepare points for upsert
            points = []
            for text, embedding, metadata, point_id in zip(
                texts, embeddings, metadatas, ids
            ):
                payload = {
                    "text": text,
                    "metadata": metadata,
                    "input_type": input_type,
                    "timestamp": str(
                        "2025-12-17T10:00:00Z"
                    ),  # In practice, use datetime.utcnow().isoformat()
                }

                point = PointStruct(id=point_id, vector=embedding, payload=payload)
                points.append(point)

            # Upsert points to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name, points=points
            )

            logger.info(
                f"Upserted {len(points)} embeddings to collection {self.collection_name}"
            )
            return ids

        except Exception as e:
            logger.error(f"Error upserting embeddings: {e}")
            raise

    def upsert_single_embedding(
        self,
        text: str,
        metadata: Optional[Dict[str, Any]] = None,
        point_id: Optional[str] = None,
        input_type: str = "search_document",
    ) -> str:
        """
        Upsert a single text embedding into Qdrant collection.

        Args:
            text: Text to embed and store
            metadata: Optional metadata dictionary
            point_id: Optional ID for the embedding. If None, will generate a UUID
            input_type: Type of input for embedding model

        Returns:
            ID of the stored embedding
        """
        if point_id is None:
            point_id = str(uuid4())

        return self.upsert_embeddings(
            texts=[text],
            metadatas=[metadata or {}],
            ids=[point_id],
            input_type=input_type,
        )[0]

    def search_similar(
        self, query: str, limit: int = 10, filters: Optional[models.Filter] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in Qdrant.

        Args:
            query: Query text to search for
            limit: Number of results to return
            filters: Optional filters to apply to the search

        Returns:
            List of search results with text, metadata, and similarity scores
        """
        try:
            # Generate embedding for the query using search_query input type
            query_embedding = self.embed_single_text(query, input_type="search_query")

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                query_filter=filters,
            )

            # Format results
            results = []
            for result in search_results:
                results.append(
                    {
                        "id": result.id,
                        "text": result.payload.get("text", ""),
                        "metadata": result.payload.get("metadata", {}),
                        "input_type": result.payload.get("input_type", ""),
                        "score": result.score,
                        "payload": result.payload,
                    }
                )

            return results

        except Exception as e:
            logger.error(f"Error searching for similar embeddings: {e}")
            raise

    def delete_embeddings_by_ids(self, point_ids: List[str]) -> bool:
        """
        Delete embeddings from Qdrant by their IDs.

        Args:
            point_ids: List of point IDs to delete

        Returns:
            True if deletion was successful
        """
        try:
            self.qdrant_client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=point_ids),
            )
            logger.info(
                f"Deleted {len(point_ids)} embeddings from collection {self.collection_name}"
            )
            return True
        except Exception as e:
            logger.error(f"Error deleting embeddings: {e}")
            return False

    def get_embedding_count(self) -> int:
        """
        Get the total count of embeddings in the collection.

        Returns:
            Total number of embeddings in the collection
        """
        try:
            count = self.qdrant_client.count(collection_name=self.collection_name)
            return count.count
        except Exception as e:
            logger.error(f"Error getting embedding count: {e}")
            return 0


def main():
    """
    Example usage of the TextEmbeddingHandler.
    """
    print("📚 Starting Text Embedding Handler...")

    try:
        # Initialize the handler
        handler = TextEmbeddingHandler()

        # Example: Process a sample book text
        sample_text = """
        Physical AI and Humanoid Robotics represent the cutting edge of artificial intelligence research,
        combining machine learning with embodied systems. This interdisciplinary field explores how
        intelligent agents can interact with the real world through sensors and actuators, creating
        systems that can perceive, reason, and act in physical environments.

        The field encompasses several key areas including embodied cognition, which suggests that
        the body plays an active role in shaping cognition. This challenges traditional approaches
        that treat intelligence as purely computational, instead emphasizing the importance of
        physical interaction with the environment.

        Deep reinforcement learning is commonly used to train humanoid robots for complex tasks,
        allowing them to learn through trial and error in simulated environments before transferring
        to real-world applications. Sim-to-real transfer remains a significant challenge due to
        the reality gap between simulated and actual environments.

        Advanced control systems are essential for humanoid robots to maintain balance and execute
        complex movements. These systems often incorporate feedback from multiple sensors including
        gyroscopes, accelerometers, and force-torque sensors to achieve stable locomotion.

        The integration of perception systems allows humanoid robots to understand their environment
        through computer vision, lidar, and other sensing modalities. This perception capability
        enables robots to navigate complex environments and interact with objects safely.
        """

        print("📦 Chunking sample text...")
        # Chunk the text to handle large texts
        text_chunks = handler.chunk_text(sample_text, chunk_size=300, overlap=50)
        print(f"Split text into {len(text_chunks)} chunks")

        # Add metadata for each chunk
        metadatas = []
        for i, chunk in enumerate(text_chunks):
            metadatas.append(
                {
                    "source": "sample_book",
                    "chunk_index": i,
                    "total_chunks": len(text_chunks),
                    "category": "introduction",
                }
            )

        print(".embedding chunks and upserting to Qdrant...")
        # Upsert the chunks with search_document input type (for storage)
        point_ids = handler.upsert_embeddings(
            texts=text_chunks,
            metadatas=metadatas,
            input_type="search_document",  # For storing document chunks
        )
        print(f"Upserted {len(point_ids)} chunks with IDs: {point_ids[:3]}...")

        # Example: Search with a query using search_query input type
        query = "How is AI used in robotics?"
        print(f"\n🔍 Searching for similar content to: '{query}'")

        search_results = handler.search_similar(query, limit=3)
        print(f"Found {len(search_results)} results:")
        for i, result in enumerate(search_results, 1):
            print(f"  {i}. Score: {result['score']:.3f}")
            print(f"     Text: {result['text'][:100]}...")
            print(f"     Metadata: {result['metadata']}")
            print()

        # Get total count
        total_count = handler.get_embedding_count()
        print(f"📊 Total embeddings in database: {total_count}")

        print("✅ Text Embedding Handler completed successfully!")

    except Exception as e:
        logger.error(f"❌ Error in text embedding handler: {e}")
        raise


if __name__ == "__main__":
    main()
