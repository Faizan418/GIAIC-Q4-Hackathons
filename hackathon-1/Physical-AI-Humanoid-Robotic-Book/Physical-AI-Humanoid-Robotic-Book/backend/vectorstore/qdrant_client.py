"""Qdrant vector database client for the RAG Chatbot API."""

import logging
from typing import Any, Dict, List, Optional

from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models

from backend.config import settings

logger = logging.getLogger(__name__)


class QdrantVectorStore:
    """Class to handle Qdrant vector database operations."""

    def __init__(self):
        # Ensure URL has proper protocol for Qdrant Cloud
        url = settings.qdrant_url
        if url and not url.startswith(("http://", "https://")):
            url = f"https://{url}"

        self.client = AsyncQdrantClient(
            url=url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False,  # Use REST API for Cloud connections
        )
        self.collection_name = settings.qdrant_collection
        self._initialized = False

    async def initialize(self):
        """Initialize the Qdrant collection."""
        try:
            # Check if collection exists
            collections = await self.client.get_collections()
            collection_exists = any(
                col.name == self.collection_name for col in collections.collections
            )

            if not collection_exists:
                # Create collection with appropriate vector size (Cohere's multilingual model uses 768 dimensions)
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1024,  # Cohere's embedding dimension
                        distance=models.Distance.COSINE,
                    ),
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

            self._initialized = True
            logger.info("Qdrant vector store initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant: {e}")
            raise

    async def store_embedding(
        self,
        embedding_id: str,
        embedding: List[float],
        content_id: str,
        module: str = "",
        chapter: str = "",
        section: str = "",
        metadata: Optional[Dict[str, Any]] = None,
        text_chunk: str = "",  # Add text chunk parameter
    ) -> str:
        """Store an embedding in Qdrant."""
        if not self._initialized:
            raise Exception("Qdrant vector store not initialized")

        try:
            point_id = embedding_id  # Use the provided embedding_id as the point ID
            payload = {
                "embedding_id": embedding_id,
                "content_id": content_id,
                "text": text_chunk,  # Store the text content
                "module": module,
                "chapter": chapter,
                "section": section,
                "created_at": "2025-12-16T10:00:00Z",  # In practice, use datetime.utcnow().isoformat()
            }
            if metadata:
                payload.update(metadata)

            await self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(id=point_id, vector=embedding, payload=payload)
                ],
                wait=True,  # Ensure write completion
            )
            return point_id
        except Exception as e:
            logger.error(f"Failed to store embedding: {e}")
            raise

    async def search_similar(
        self,
        query_vector: List[float],
        top_k: int = 5,
        module_filter: Optional[str] = None,
        chapter_filter: Optional[str] = None,
        section_filter: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """Search for similar embeddings in Qdrant with optional filters."""
        if not self._initialized:
            raise Exception("Qdrant vector store not initialized")

        try:
            # Build filters if any are provided
            filters = []
            if module_filter:
                filters.append(
                    models.FieldCondition(
                        key="module", match=models.MatchValue(value=module_filter)
                    )
                )
            if chapter_filter:
                filters.append(
                    models.FieldCondition(
                        key="chapter", match=models.MatchValue(value=chapter_filter)
                    )
                )
            if section_filter:
                filters.append(
                    models.FieldCondition(
                        key="section", match=models.MatchValue(value=section_filter)
                    )
                )

            # Create the filter condition
            filter_condition = None
            if filters:
                if len(filters) == 1:
                    filter_condition = models.Filter(must=[filters[0]])
                else:
                    filter_condition = models.Filter(must=filters)

            search_results = await self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                query_filter=filter_condition,
                with_payload=True,  # Ensure payload is returned
            )

            results = []
            for result in search_results:
                payload = result.payload or {}
                results.append(
                    {
                        "embedding_id": result.id,
                        "content_id": payload.get("content_id"),
                        "text_chunk": payload.get(
                            "text", ""
                        ),  # Include the text content
                        "module": payload.get("module"),
                        "chapter": payload.get("chapter"),
                        "section": payload.get("section"),
                        "relevance_score": result.score,
                        "metadata": payload,
                    }
                )

            return results
        except Exception as e:
            logger.error(f"Failed to search similar embeddings: {e}")
            raise

    async def delete_embedding(self, point_id: str):
        """Delete an embedding from Qdrant."""
        if not self._initialized:
            raise Exception("Qdrant vector store not initialized")

        try:
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=[point_id]),
            )
        except Exception as e:
            logger.error(f"Failed to delete embedding: {e}")
            raise

    async def delete_embeddings_by_content_id(self, content_id: str):
        """Delete all embeddings associated with a content ID."""
        if not self._initialized:
            raise Exception("Qdrant vector store not initialized")

        try:
            # Delete embeddings by filtering on content_id
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_id", match=models.MatchValue(value=content_id)
                        )
                    ]
                ),
            )
        except Exception as e:
            logger.error(f"Failed to delete embeddings by content ID: {e}")
            raise

    async def get_embedding(self, point_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific embedding from Qdrant."""
        if not self._initialized:
            raise Exception("Qdrant vector store not initialized")

        try:
            points = await self.client.retrieve(
                collection_name=self.collection_name, ids=[point_id]
            )

            if points:
                point = points[0]
                return {
                    "id": point.id,
                    "vector": point.vector,
                    "payload": point.payload,
                }
            return None
        except Exception as e:
            logger.error(f"Failed to get embedding: {e}")
            raise


# Global instance
qdrant_client = QdrantVectorStore()
