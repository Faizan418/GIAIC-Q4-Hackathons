"""Embedding service for the RAG Chatbot API."""

import logging
import uuid
from datetime import datetime
from typing import List, Optional

from backend.db.neon_postgres import db
from backend.embeddings.cohere_embed import CohereEmbeddingService
from backend.models.content import ContentChunk, TextbookContent
from backend.models.embedding import EmbeddingModel
from backend.utils.exceptions import ContentRetrievalError, EmbeddingGenerationError
from backend.utils.logging import rag_logger
from backend.vectorstore.qdrant_client import qdrant_client

logger = logging.getLogger(__name__)


class EmbeddingService:
    """Service for handling embedding generation, storage, and retrieval."""

    def __init__(self):
        self.cohere_service = CohereEmbeddingService()
        self.db = db
        self.qdrant = qdrant_client

    async def generate_embedding(
        self,
        text: str,
        model_name: str = "embed-english-v3.0",
        input_type: str = "search_document",
    ) -> List[float]:
        """Generate embedding for the given text using Cohere."""
        try:
            # Use the async method from CohereEmbeddingService
            embedding = await self.cohere_service.generate_embedding(
                text, model_name, input_type
            )
            rag_logger.log_embedding_generation(
                text[:100] + "..." if len(text) > 100 else text, model_name
            )
            return embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {str(e)}")
            raise EmbeddingGenerationError(f"Failed to generate embedding: {str(e)}")

    async def generate_embeddings_batch(
        self,
        texts: List[str],
        model_name: str = "embed-english-v3.0",
        input_type: str = "search_document",
    ) -> List[List[float]]:
        """Generate embeddings for a batch of texts using Cohere."""
        try:
            # Use the async method from CohereEmbeddingService
            embeddings = await self.cohere_service.generate_embeddings_batch(
                texts, model_name, input_type
            )
            rag_logger.log_embedding_generation_batch(len(texts), model_name)
            return embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings batch: {str(e)}")
            raise EmbeddingGenerationError(
                f"Failed to generate embeddings batch: {str(e)}"
            )

    async def create_embedding_record(
        self,
        content_id: str,
        embedding: List[float],
        model_name: str,
        module: str = "",
        chapter: str = "",
        section: str = "",
        metadata: Optional[dict] = None,
    ) -> EmbeddingModel:
        """Create an embedding record in the database."""
        try:
            embedding_id = str(uuid.uuid4())
            embedding_record = EmbeddingModel(
                embedding_id=embedding_id,
                content_id=content_id,
                embedding=embedding,
                model_name=model_name,
                module=module,
                chapter=chapter,
                section=section,
                metadata=metadata or {},
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
            )

            # Store in Postgres
            await self.db.insert_embedding(embedding_record)

            # Store in Qdrant
            await self.qdrant.store_embedding(
                embedding_id=embedding_id,
                embedding=embedding,
                content_id=content_id,
                module=module,
                chapter=chapter,
                section=section,
                metadata=metadata or {},
            )

            rag_logger.log_embedding_storage(embedding_id, content_id, model_name)
            return embedding_record
        except Exception as e:
            logger.error(f"Error creating embedding record: {str(e)}")
            raise EmbeddingGenerationError(
                f"Failed to create embedding record: {str(e)}"
            )

    async def create_embeddings_for_content(
        self, content: TextbookContent
    ) -> List[EmbeddingModel]:
        """Create embeddings for all chunks of a content piece."""
        try:
            # First, chunk the content
            chunks = self.chunk_content(content)

            # Generate embeddings for all chunks
            texts = [chunk.text_chunk for chunk in chunks]
            embeddings = await self.generate_embeddings_batch(texts)

            # Create embedding records for each chunk
            embedding_records = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                embedding_record = await self.create_embedding_record(
                    content_id=content.content_id,
                    embedding=embedding,
                    model_name="embed-english-v3.0",
                    module=content.module,
                    chapter=content.chapter,
                    section=content.section,
                    metadata={
                        "chunk_index": chunk.chunk_index,
                        "total_chunks": len(chunks),
                        "original_length": len(content.text),
                    },
                )
                embedding_records.append(embedding_record)

            rag_logger.log_content_embedding(content.content_id, len(chunks))
            return embedding_records
        except Exception as e:
            logger.error(
                f"Error creating embeddings for content {content.content_id}: {str(e)}"
            )
            raise EmbeddingGenerationError(
                f"Failed to create embeddings for content: {str(e)}"
            )

    def chunk_content(
        self, content: TextbookContent, max_chunk_size: int = 512, overlap: int = 50
    ) -> List[ContentChunk]:
        """Chunk content into smaller pieces for embedding."""
        text = content.text
        chunks = []

        start = 0
        chunk_index = 0

        while start < len(text):
            # Determine the end position for this chunk
            end = start + max_chunk_size

            # If this is not the last chunk, try to break at a sentence boundary
            if end < len(text):
                # Look for a sentence boundary near the end
                sentence_end = text.rfind(". ", start + max_chunk_size - 100, end)
                if sentence_end != -1 and sentence_end > start:
                    end = sentence_end + 1  # Include the period
                else:
                    # If no sentence boundary found, look for a paragraph break
                    para_end = text.rfind("\n\n", start + max_chunk_size - 100, end)
                    if para_end != -1 and para_end > start:
                        end = para_end
                    # Otherwise, just break at max_chunk_size

            # Create the chunk
            chunk_text = text[start:end].strip()
            if chunk_text:  # Only add non-empty chunks
                chunk = ContentChunk(
                    content_id=content.content_id,
                    text_chunk=chunk_text,
                    chunk_index=chunk_index,
                    module=content.module,
                    chapter=content.chapter,
                    section=content.section,
                )
                chunks.append(chunk)

            # Move to the next chunk position with overlap
            start = end - overlap if end < len(text) else end
            chunk_index += 1

        return chunks

    async def retrieve_similar_embeddings(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        module: Optional[str] = None,
        chapter: Optional[str] = None,
        section: Optional[str] = None,
    ) -> List[EmbeddingModel]:
        """Retrieve similar embeddings from Qdrant."""
        try:
            # Query Qdrant for similar embeddings
            similar_results = await self.qdrant.search_similar(
                query_embedding=query_embedding,
                top_k=top_k,
                module_filter=module,
                chapter_filter=chapter,
                section_filter=section,
            )

            # Get the full embedding records from Postgres
            embedding_records = []
            for result in similar_results:
                embedding_record = await self.db.get_embedding_by_id(
                    result.embedding_id
                )
                if embedding_record:
                    embedding_records.append(embedding_record)

            rag_logger.log_embedding_retrieval(len(embedding_records), top_k)
            return embedding_records
        except Exception as e:
            logger.error(f"Error retrieving similar embeddings: {str(e)}")
            raise ContentRetrievalError(
                f"Failed to retrieve similar embeddings: {str(e)}"
            )

    async def process_and_store_content(self, content: TextbookContent) -> List[str]:
        """Process content by chunking, embedding, and storing in both databases."""
        try:
            # Store the content in Postgres first
            await self.db.insert_content(content)

            # Create embeddings for the content
            embedding_records = await self.create_embeddings_for_content(content)

            # Return the IDs of created embeddings
            embedding_ids = [record.embedding_id for record in embedding_records]

            logger.info(
                f"Successfully processed and stored content {content.content_id} with {len(embedding_ids)} embeddings"
            )
            return embedding_ids
        except Exception as e:
            logger.error(
                f"Error processing and storing content {content.content_id}: {str(e)}"
            )
            raise EmbeddingGenerationError(
                f"Failed to process and store content: {str(e)}"
            )

    async def delete_embeddings_for_content(self, content_id: str) -> bool:
        """Delete all embeddings associated with a content piece."""
        try:
            # Get all embedding IDs for this content
            embedding_ids = await self.db.get_embedding_ids_by_content_id(content_id)

            # Delete from Qdrant
            for embedding_id in embedding_ids:
                await self.qdrant.delete_embedding(embedding_id)

            # Delete from Postgres
            await self.db.delete_embeddings_by_content_id(content_id)

            logger.info(
                f"Deleted {len(embedding_ids)} embeddings for content {content_id}"
            )
            return True
        except Exception as e:
            logger.error(
                f"Error deleting embeddings for content {content_id}: {str(e)}"
            )
            return False


# Singleton instance
embedding_service = EmbeddingService()
