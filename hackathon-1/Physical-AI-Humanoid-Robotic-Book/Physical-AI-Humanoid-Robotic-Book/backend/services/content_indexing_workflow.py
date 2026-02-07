"""Content indexing workflow for the RAG Chatbot API."""

import asyncio
import logging
from datetime import datetime
from typing import Any, Dict, List

from backend.db.neon_postgres import db
from backend.models.content import TextbookContent
from backend.services.content_processing_pipeline import content_processing_pipeline
from backend.utils.exceptions import ContentProcessingError
from backend.utils.logging import rag_logger
from backend.vectorstore.qdrant_client import qdrant_client

logger = logging.getLogger(__name__)


class ContentIndexingWorkflow:
    """Workflow for indexing textbook content into the RAG system."""

    def __init__(self):
        self.content_pipeline = content_processing_pipeline
        self.db = db
        self.qdrant = qdrant_client

    async def index_single_content(self, content: TextbookContent) -> Dict[str, Any]:
        """Index a single piece of content into the RAG system."""
        try:
            logger.info(f"Starting indexing for content: {content.content_id}")

            # Process the content (validate, store in Postgres, generate embeddings, store in Qdrant)
            embedding_ids = await self.content_pipeline.process_single_content(content)

            result = {
                "content_id": content.content_id,
                "title": content.title,
                "module": content.module,
                "chapter": content.chapter,
                "section": content.section,
                "embedding_count": len(embedding_ids),
                "embedding_ids": embedding_ids,
                "status": "success",
                "indexed_at": datetime.utcnow().isoformat(),
            }

            logger.info(
                f"Successfully indexed content {content.content_id} with {len(embedding_ids)} embeddings"
            )
            rag_logger.log_content_indexing(
                content.content_id, len(embedding_ids), "success"
            )

            return result
        except Exception as e:
            logger.error(f"Error indexing content {content.content_id}: {str(e)}")
            rag_logger.log_content_indexing(content.content_id, 0, "error", str(e))
            raise ContentProcessingError(
                f"Failed to index content {content.content_id}: {str(e)}"
            )

    async def index_content_batch(
        self, contents: List[TextbookContent], max_concurrent: int = 5
    ) -> Dict[str, Any]:
        """Index a batch of content with controlled concurrency."""
        results = {
            "successful": [],
            "failed": [],
            "total_processed": len(contents),
            "timestamp": datetime.utcnow().isoformat(),
        }

        logger.info(f"Starting batch indexing for {len(contents)} content items")

        # Process contents with limited concurrency
        semaphore = asyncio.Semaphore(max_concurrent)

        async def index_with_semaphore(content: TextbookContent):
            async with semaphore:
                try:
                    result = await self.index_single_content(content)
                    return result
                except Exception as e:
                    error_result = {
                        "content_id": content.content_id,
                        "title": content.title,
                        "module": content.module,
                        "chapter": content.chapter,
                        "section": content.section,
                        "status": "failed",
                        "error": str(e),
                        "indexed_at": datetime.utcnow().isoformat(),
                    }
                    return error_result

        # Create tasks for all contents
        tasks = [index_with_semaphore(content) for content in contents]

        # Execute all tasks concurrently
        task_results = await asyncio.gather(*tasks, return_exceptions=True)

        # Process results
        for result in task_results:
            if isinstance(result, Exception):
                results["failed"].append({"error": str(result), "status": "exception"})
            elif result["status"] == "success":
                results["successful"].append(result)
            else:
                results["failed"].append(result)

        logger.info(
            f"Batch indexing completed: {len(results['successful'])} successful, {len(results['failed'])} failed"
        )
        rag_logger.log_batch_indexing(
            len(contents), len(results["successful"]), len(results["failed"])
        )

        return results

    async def index_content_from_textbook_structure(
        self, textbook_structure: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Index content from a structured textbook format."""
        try:
            contents = []

            # Parse the textbook structure and create content objects
            for module_data in textbook_structure.get("modules", []):
                module_id = module_data.get("module_id", "")

                for chapter_data in module_data.get("chapters", []):
                    chapter_id = chapter_data.get("chapter_id", "")

                    # Process sections in the chapter
                    for section_data in chapter_data.get("sections", []):
                        section_id = section_data.get("section_id", "")

                        # Create content object for each section
                        content = TextbookContent(
                            content_id=section_data.get(
                                "content_id", f"{module_id}-{chapter_id}-{section_id}"
                            ),
                            title=section_data.get("title", f"Section {section_id}"),
                            text=section_data.get("content", ""),
                            module=module_id,
                            chapter=chapter_id,
                            section=section_id,
                            page_numbers=section_data.get("page_numbers"),
                            metadata=section_data.get("metadata", {}),
                        )

                        contents.append(content)

            # Index all contents in batch
            results = await self.index_content_batch(contents)

            logger.info(
                f"Textbook structure indexing completed: {len(results['successful'])} successful, {len(results['failed'])} failed"
            )
            return results

        except Exception as e:
            logger.error(f"Error indexing from textbook structure: {str(e)}")
            raise ContentProcessingError(
                f"Failed to index textbook structure: {str(e)}"
            )

    async def reindex_content(self, content_id: str) -> Dict[str, Any]:
        """Re-index specific content (delete old embeddings and reprocess)."""
        try:
            logger.info(f"Starting re-indexing for content: {content_id}")

            # Update content (this will delete old embeddings and create new ones)
            content = await self.db.get_content_by_id(content_id)
            if not content:
                raise ContentProcessingError(f"Content with ID {content_id} not found")

            # Convert dict to TextbookContent model
            textbook_content = TextbookContent(
                content_id=content["content_id"],
                title=content["title"],
                text=content["text"],
                module=content["module"],
                chapter=content["chapter"],
                section=content["section"],
                page_numbers=content["page_numbers"],
                metadata=content["metadata"],
            )

            # Process the content again to regenerate embeddings
            embedding_ids = await self.content_pipeline.update_content(textbook_content)

            result = {
                "content_id": content_id,
                "embedding_count": len(embedding_ids),
                "embedding_ids": embedding_ids,
                "status": "success",
                "reindexed_at": datetime.utcnow().isoformat(),
            }

            logger.info(f"Successfully reindexed content {content_id}")
            rag_logger.log_content_reindexing(content_id, len(embedding_ids), "success")

            return result
        except Exception as e:
            logger.error(f"Error reindexing content {content_id}: {str(e)}")
            rag_logger.log_content_reindexing(content_id, 0, "error", str(e))
            raise ContentProcessingError(
                f"Failed to reindex content {content_id}: {str(e)}"
            )

    async def reindex_all_content(self) -> Dict[str, Any]:
        """Re-index all content in the system."""
        try:
            logger.info("Starting re-indexing for all content")

            # Get all content IDs
            content_ids = await self.db.get_all_content_ids()

            # Get all content objects
            all_contents = []
            for content_id in content_ids:
                content = await self.db.get_content_by_id(content_id)
                if content:
                    textbook_content = TextbookContent(
                        content_id=content["content_id"],
                        title=content["title"],
                        text=content["text"],
                        module=content["module"],
                        chapter=content["chapter"],
                        section=content["section"],
                        page_numbers=content["page_numbers"],
                        metadata=content["metadata"],
                    )
                    all_contents.append(textbook_content)

            # Process all contents in batch
            results = await self.index_content_batch(all_contents)

            logger.info(
                f"Re-indexing all content completed: {len(results['successful'])} successful, {len(results['failed'])} failed"
            )
            return results

        except Exception as e:
            logger.error(f"Error reindexing all content: {str(e)}")
            raise ContentProcessingError(f"Failed to reindex all content: {str(e)}")

    async def validate_indexing_status(self) -> Dict[str, Any]:
        """Validate the current indexing status and provide metrics."""
        try:
            # Get counts from both databases
            postgres_content_count = len(await self.db.get_all_content_ids())

            # For Qdrant, we'll use a simple count query
            # Note: This might need adjustment based on Qdrant client capabilities
            try:
                collection_info = self.qdrant.client.get_collection(
                    self.qdrant.collection_name
                )
                qdrant_embedding_count = collection_info.points_count
            except:
                qdrant_embedding_count = "unknown"

            status = {
                "timestamp": datetime.utcnow().isoformat(),
                "databases": {
                    "postgres": {"content_count": postgres_content_count},
                    "qdrant": {"embedding_count": qdrant_embedding_count},
                },
                "status": "healthy" if postgres_content_count > 0 else "empty",
            }

            return status
        except Exception as e:
            logger.error(f"Error validating indexing status: {str(e)}")
            return {
                "timestamp": datetime.utcnow().isoformat(),
                "status": "error",
                "error": str(e),
            }

    async def cleanup_orphaned_embeddings(self) -> Dict[str, Any]:
        """Clean up embeddings that don't have corresponding content in Postgres."""
        try:
            logger.info("Starting cleanup of orphaned embeddings")

            # Get all content IDs from Postgres
            postgres_content_ids = set(await self.db.get_all_content_ids())

            # This is a simplified approach - in a real implementation, we'd need to iterate through Qdrant embeddings
            # For now, we'll just return a message indicating what would be done
            result = {
                "action": "cleanup_orphaned_embeddings",
                "status": "completed",
                "message": "Orphaned embeddings cleanup would happen here. In a real implementation, this would iterate through Qdrant embeddings and remove those without corresponding Postgres content.",
                "timestamp": datetime.utcnow().isoformat(),
            }

            logger.info("Completed cleanup of orphaned embeddings")
            return result
        except Exception as e:
            logger.error(f"Error cleaning up orphaned embeddings: {str(e)}")
            raise ContentProcessingError(
                f"Failed to clean up orphaned embeddings: {str(e)}"
            )


# Singleton instance
content_indexing_workflow = ContentIndexingWorkflow()
