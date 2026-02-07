#!/usr/bin/env python3
"""
Book Embedding Service for Qdrant Vector Database
This service handles the creation and storage of book content embeddings in Qdrant
specifically for the Physical AI & Humanoid Robotics textbook project.
"""

import logging
import os
from pathlib import Path
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


class BookEmbeddingService:
    def __init__(
        self,
        cohere_api_key: Optional[str] = None,
        qdrant_url: Optional[str] = None,
        qdrant_api_key: Optional[str] = None,
    ):
        """
        Initialize the book embedding service with Cohere and Qdrant clients
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

        # Collection name for book embeddings
        self.collection_name = "book_embeddings"

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

    def _chunk_text(
        self, text: str, chunk_size: int = 1000, overlap: int = 100
    ) -> List[str]:
        """
        Split text into overlapping chunks to preserve context
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

    def create_embedding(self, text: str) -> List[float]:
        """
        Create a single embedding for the given text
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-english-v3.0",  # Using Cohere's latest embedding model
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
            response = self.cohere_client.embed(texts=texts, model="embed-english-v3.0")
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            logger.error(f"Error generating batch embeddings: {e}")
            raise

    def store_book_content(
        self,
        title: str,
        content: str,
        chapter: Optional[str] = None,
        section: Optional[str] = None,
        page_numbers: Optional[str] = None,
    ) -> List[str]:
        """
        Store book content as embeddings in Qdrant with proper metadata
        """
        logger.info(f"Storing book content: {title}")

        # Chunk the content to handle large texts
        text_chunks = self._chunk_text(content)
        point_ids = []

        # Process in batches to respect API limits
        batch_size = min(96, len(text_chunks))  # Cohere API limit

        for i in range(0, len(text_chunks), batch_size):
            batch_chunks = text_chunks[i : i + batch_size]

            try:
                # Generate embeddings for the batch
                embeddings = self.batch_create_embeddings(batch_chunks)

                # Prepare points with metadata
                points = []
                for j, (chunk, embedding) in enumerate(zip(batch_chunks, embeddings)):
                    # Create a unique ID for this chunk
                    chunk_id = f"{title.replace(' ', '_')}_{i + j}_{str(uuid4())[:8]}"

                    # Prepare metadata
                    metadata = {
                        "title": title,
                        "content": chunk,
                        "chunk_index": i + j,
                        "total_chunks": len(text_chunks),
                        "type": "book_content",
                    }

                    if chapter:
                        metadata["chapter"] = chapter
                    if section:
                        metadata["section"] = section
                    if page_numbers:
                        metadata["page_numbers"] = page_numbers

                    # Create point
                    point = PointStruct(id=chunk_id, vector=embedding, payload=metadata)

                    points.append(point)
                    point_ids.append(chunk_id)

                # Store batch in Qdrant
                self.qdrant_client.upsert(
                    collection_name=self.collection_name, points=points
                )

                logger.info(f"Stored batch of {len(points)} book content chunks")

            except Exception as e:
                logger.error(f"Error storing batch of book content: {e}")
                raise

        logger.info(f"Successfully stored {len(point_ids)} chunks for '{title}'")
        return point_ids

    def store_book_from_file(
        self, file_path: str, title: str, chapter: Optional[str] = None
    ) -> List[str]:
        """
        Store book content from a file in Qdrant
        """
        try:
            with open(file_path, "r", encoding="utf-8") as file:
                content = file.read()

            return self.store_book_content(title, content, chapter)

        except Exception as e:
            logger.error(f"Error reading file {file_path}: {e}")
            raise

    def store_book_from_directory(
        self, directory_path: str, title_prefix: str = ""
    ) -> Dict[str, List[str]]:
        """
        Store all book content from a directory in Qdrant
        """
        directory = Path(directory_path)
        results = {}

        # Process markdown and text files
        for file_path in directory.rglob("*.md"):
            try:
                # Use the file name as the chapter/section identifier
                file_title = f"{title_prefix} - {file_path.stem}"
                chapter = file_path.stem

                logger.info(f"Processing file: {file_path}")

                file_results = self.store_book_from_file(
                    str(file_path), file_title, chapter
                )

                results[str(file_path)] = file_results
                logger.info(f"Completed processing: {file_path}")

            except Exception as e:
                logger.error(f"Error processing file {file_path}: {e}")
                continue

        # Also process any .txt files
        for file_path in directory.rglob("*.txt"):
            try:
                # Use the file name as the chapter/section identifier
                file_title = f"{title_prefix} - {file_path.stem}"
                chapter = file_path.stem

                logger.info(f"Processing file: {file_path}")

                file_results = self.store_book_from_file(
                    str(file_path), file_title, chapter
                )

                results[str(file_path)] = file_results
                logger.info(f"Completed processing: {file_path}")

            except Exception as e:
                logger.error(f"Error processing file {file_path}: {e}")
                continue

        return results

    def search_book_content(
        self, query: str, limit: int = 10, filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for book content in Qdrant using vector similarity
        """
        try:
            # Generate embedding for the query
            query_embedding = self.create_embedding(query)

            # Prepare filters if provided
            search_filter = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=f"metadata.{key}", match=models.MatchValue(value=value)
                        )
                    )

                if filter_conditions:
                    search_filter = models.Filter(must=filter_conditions)

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=search_filter,
                limit=limit,
            )

            # Format results
            results = []
            for result in search_results:
                payload = result.payload or {}
                results.append(
                    {
                        "id": result.id,
                        "content": payload.get("content", ""),
                        "title": payload.get("title", ""),
                        "chapter": payload.get("chapter", ""),
                        "section": payload.get("section", ""),
                        "page_numbers": payload.get("page_numbers", ""),
                        "score": result.score,
                        "chunk_index": payload.get("chunk_index", 0),
                        "total_chunks": payload.get("total_chunks", 1),
                    }
                )

            return results

        except Exception as e:
            logger.error(f"Error searching for book content: {e}")
            raise

    def get_book_embedding_count(self) -> int:
        """
        Get the total count of book embeddings in the collection
        """
        try:
            count = self.qdrant_client.count(collection_name=self.collection_name)
            return count.count
        except Exception as e:
            logger.error(f"Error getting book embedding count: {e}")
            return 0

    def get_books_in_collection(self) -> List[str]:
        """
        Get a list of unique book titles in the collection
        """
        try:
            # Get all points with title metadata
            points, _ = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                limit=10000,  # Adjust as needed
                with_payload=True,
            )

            titles = set()
            for point in points:
                payload = point.payload or {}
                if "title" in payload:
                    titles.add(payload["title"])

            return list(titles)
        except Exception as e:
            logger.error(f"Error getting books in collection: {e}")
            return []


def main():
    """
    Example usage of the Book Embedding Service
    """
    print("📚 Starting Book Embedding Service...")

    try:
        # Initialize the service
        service = BookEmbeddingService()

        # Example: Store a sample book section
        sample_content = """
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
        """

        print("📦 Storing sample book content...")
        point_ids = service.store_book_content(
            title="Physical AI and Humanoid Robotics - Introduction",
            content=sample_content,
            chapter="Chapter 1",
            section="1.1",
            page_numbers="1-15",
        )

        print(f"✅ Stored {len(point_ids)} content chunks with IDs: {point_ids[:3]}...")

        # Search for relevant content
        print("\n🔍 Searching for relevant content...")
        search_results = service.search_book_content(
            "embodied cognition in robotics", limit=3
        )

        print(f"Found {len(search_results)} results:")
        for i, result in enumerate(search_results, 1):
            print(f"  {i}. Score: {result['score']:.3f}")
            print(f"     Title: {result['title']}")
            print(f"     Chapter: {result['chapter']}")
            print(f"     Content: {result['content'][:100]}...")
            print()

        # Get total count
        total_count = service.get_book_embedding_count()
        print(f"📊 Total book embeddings in database: {total_count}")

        # Get unique books
        books = service.get_books_in_collection()
        print(f"📚 Books in collection: {books}")

        print("\n🎉 Book Embedding Service completed successfully!")

    except Exception as e:
        logger.error(f"❌ Error in book embedding service: {e}")
        raise


if __name__ == "__main__":
    main()
