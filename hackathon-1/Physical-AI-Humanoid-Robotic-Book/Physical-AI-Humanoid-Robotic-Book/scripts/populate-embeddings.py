#!/usr/bin/env python3
"""
Script to populate the vector database with embeddings from the textbook content.
This script reads the Docusaurus documentation files and generates embeddings
using the Cohere API, then stores them in Qdrant for RAG functionality.
"""

import asyncio
import logging
import os
import sys
from pathlib import Path
from typing import Any, Dict, List

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

try:
    import cohere
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
except ImportError as e:
    print(f"Required packages not found: {e}")
    print("Please install: pip install cohere qdrant-client")
    sys.exit(1)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class EmbeddingPopulator:
    def __init__(self):
        self.cohere_api_key = os.getenv("COHERE_API_KEY")
        self.qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        # Initialize clients
        self.cohere_client = cohere.Client(self.cohere_api_key)
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url, api_key=self.qdrant_api_key, prefer_grpc=False
        )

        # Collection name for embeddings - should match config
        self.collection_name = os.getenv("QDRANT_COLLECTION", "Book-Embedding")

    def read_docusaurus_docs(self, docs_path: str) -> List[Dict[str, Any]]:
        """
        Read all markdown files from the Docusaurus docs directory
        """
        docs_path = Path(docs_path)
        documents = []

        for md_file in docs_path.rglob("*.md"):
            try:
                with open(md_file, "r", encoding="utf-8") as f:
                    content = f.read()

                # Create document with metadata
                doc = {
                    "id": str(md_file.relative_to(docs_path)),
                    "content": content,
                    "title": md_file.stem,
                    "path": str(md_file),
                    "source": "docusaurus",
                }
                documents.append(doc)
                logger.info(f"Read document: {doc['id']}")

            except Exception as e:
                logger.error(f"Error reading {md_file}: {e}")

        return documents

    def chunk_text(self, text: str, chunk_size: int = 1000) -> List[str]:
        """
        Split text into chunks of specified size
        """
        chunks = []
        for i in range(0, len(text), chunk_size):
            chunk = text[i : i + chunk_size]
            chunks.append(chunk)
        return chunks

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings using Cohere API
        """
        try:
            response = self.cohere_client.embed(
                texts=texts,
                model="embed-multilingual-v2.0",  # Using Cohere's multilingual model (768 dim)
                input_type="search_document",  # Optimize for search
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            raise

    def create_qdrant_collection(self):
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
                        size=768,  # Cohere multilingual embedding dimension
                        distance=models.Distance.COSINE,
                    ),
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {e}")
            raise

    def populate_embeddings(self, docs_path: str):
        """
        Main method to populate embeddings from docs
        """
        logger.info("Starting embedding population process...")

        # Read documents
        documents = self.read_docusaurus_docs(docs_path)
        logger.info(f"Found {len(documents)} documents to process")

        # Prepare for embedding generation
        all_chunks_with_metadata = []

        for doc in documents:
            # Split content into chunks
            chunks = self.chunk_text(doc["content"])

            for i, chunk in enumerate(chunks):
                chunk_with_metadata = {
                    "id": f"{doc['id']}_chunk_{i}",
                    "text": chunk,
                    "metadata": {
                        "title": doc["title"],
                        "path": doc["path"],
                        "source": doc["source"],
                        "chunk_index": i,
                    },
                }
                all_chunks_with_metadata.append(chunk_with_metadata)

        logger.info(
            f"Prepared {len(all_chunks_with_metadata)} text chunks for embedding"
        )

        # Create Qdrant collection
        self.create_qdrant_collection()

        # Process in batches to avoid API limits
        batch_size = 96  # Cohere's API limit is 96 texts per request
        total_processed = 0

        for i in range(0, len(all_chunks_with_metadata), batch_size):
            batch = all_chunks_with_metadata[i : i + batch_size]

            # Extract texts for embedding
            texts = [chunk["text"] for chunk in batch]

            try:
                # Generate embeddings
                embeddings = self.generate_embeddings(texts)

                # Prepare points for Qdrant
                points = []
                for j, (chunk, embedding) in enumerate(zip(batch, embeddings)):
                    point = models.PointStruct(
                        id=chunk["id"],
                        vector=embedding,
                        payload={"text": chunk["text"], "metadata": chunk["metadata"]},
                    )
                    points.append(point)

                # Upload to Qdrant
                self.qdrant_client.upsert(
                    collection_name=self.collection_name, points=points
                )

                total_processed += len(batch)
                logger.info(
                    f"Processed batch: {total_processed}/{len(all_chunks_with_metadata)} chunks"
                )

            except Exception as e:
                logger.error(f"Error processing batch: {e}")
                raise

        logger.info(
            f"Successfully populated {total_processed} embeddings to Qdrant collection: {self.collection_name}"
        )

    def verify_embeddings(self) -> int:
        """
        Verify the embeddings by checking the count in Qdrant
        """
        try:
            count = self.qdrant_client.count(collection_name=self.collection_name)
            logger.info(f"Verification: {count.count} embeddings in collection")
            return count.count
        except Exception as e:
            logger.error(f"Error verifying embeddings: {e}")
            return 0


def main():
    """
    Main function to run the embedding population
    """
    try:
        populator = EmbeddingPopulator()

        # Path to Docusaurus docs
        docs_path = os.path.join(
            os.path.dirname(__file__), "..", "docusaurus-textbook", "docs"
        )

        if not os.path.exists(docs_path):
            logger.error(f"Docs path does not exist: {docs_path}")
            sys.exit(1)

        # Populate embeddings
        populator.populate_embeddings(docs_path)

        # Verify the results
        count = populator.verify_embeddings()
        logger.info(f"Embedding population completed successfully with {count} entries")

    except Exception as e:
        logger.error(f"Error during embedding population: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
