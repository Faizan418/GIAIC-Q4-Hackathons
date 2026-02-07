"""Content processing pipeline for the RAG Chatbot API."""

import asyncio
import logging
import uuid
from typing import List

from backend.db.neon_postgres import db
from backend.models.content import TextbookContent
from backend.services.embedding_service import embedding_service
from backend.utils.exceptions import ContentProcessingError, ValidationError
from backend.utils.logging import rag_logger

logger = logging.getLogger(__name__)


class ContentProcessingPipeline:
    """Pipeline for processing textbook content: ingestion, chunking, embedding, and storage."""

    def __init__(self):
        self.embedding_service = embedding_service
        self.db = db

    async def validate_content(self, content: TextbookContent) -> bool:
        """Validate content before processing."""
        errors = []

        if not content.content_id:
            errors.append("Content ID is required")

        if not content.title:
            errors.append("Content title is required")

        if not content.text:
            errors.append("Content text is required")

        if not content.module:
            errors.append("Module identifier is required")

        if not content.chapter:
            errors.append("Chapter identifier is required")

        if not content.section:
            errors.append("Section identifier is required")

        if len(content.text) < 10:
            errors.append("Content text is too short (minimum 10 characters)")

        if len(content.text) > 100000:  # 100k characters max
            errors.append("Content text is too long (maximum 100,000 characters)")

        if errors:
            raise ValidationError(f"Content validation failed: {'; '.join(errors)}")

        return True

    async def process_single_content(self, content: TextbookContent) -> List[str]:
        """Process a single content piece: validate, store, embed, and index."""
        try:
            # Validate content
            await self.validate_content(content)

            # Log the start of processing
            rag_logger.log_content_processing_start(
                content.content_id, content.title, len(content.text)
            )

            # Process and store the content (this handles chunking, embedding, and storage)
            embedding_ids = await self.embedding_service.process_and_store_content(
                content
            )

            # Log successful processing
            rag_logger.log_content_processing_complete(
                content.content_id, len(embedding_ids)
            )

            logger.info(
                f"Successfully processed content {content.content_id} with {len(embedding_ids)} embeddings"
            )
            return embedding_ids

        except ValidationError as e:
            logger.error(
                f"Validation error processing content {content.content_id}: {str(e)}"
            )
            rag_logger.log_content_processing_error(
                content.content_id, "validation_error", str(e)
            )
            raise
        except Exception as e:
            logger.error(f"Error processing content {content.content_id}: {str(e)}")
            rag_logger.log_content_processing_error(
                content.content_id, "processing_error", str(e)
            )
            raise ContentProcessingError(
                f"Failed to process content {content.content_id}: {str(e)}"
            )

    async def process_content_batch(
        self, contents: List[TextbookContent], max_concurrent: int = 5
    ) -> dict:
        """Process a batch of content pieces with controlled concurrency."""
        results = {"successful": [], "failed": [], "errors": []}

        # Process contents with limited concurrency
        semaphore = asyncio.Semaphore(max_concurrent)

        async def process_with_semaphore(content: TextbookContent):
            async with semaphore:
                try:
                    embedding_ids = await self.process_single_content(content)
                    return {
                        "content_id": content.content_id,
                        "embedding_ids": embedding_ids,
                        "status": "success",
                    }
                except Exception as e:
                    error_msg = (
                        f"Error processing content {content.content_id}: {str(e)}"
                    )
                    logger.error(error_msg)
                    return {
                        "content_id": content.content_id,
                        "error": str(e),
                        "status": "failed",
                    }

        # Create tasks for all contents
        tasks = [process_with_semaphore(content) for content in contents]

        # Execute all tasks concurrently
        task_results = await asyncio.gather(*tasks, return_exceptions=True)

        # Process results
        for result in task_results:
            if isinstance(result, Exception):
                results["errors"].append(str(result))
            elif result["status"] == "success":
                results["successful"].append(
                    {
                        "content_id": result["content_id"],
                        "embedding_ids": result["embedding_ids"],
                    }
                )
            else:
                results["failed"].append(
                    {"content_id": result["content_id"], "error": result["error"]}
                )

        # Log batch processing summary
        rag_logger.log_content_batch_processing(
            len(contents), len(results["successful"]), len(results["failed"])
        )

        logger.info(
            f"Batch processing completed: {len(results['successful'])} successful, {len(results['failed'])} failed"
        )
        return results

    async def update_content(self, content: TextbookContent) -> List[str]:
        """Update existing content: delete old embeddings, process new content."""
        try:
            # Validate content
            await self.validate_content(content)

            # Delete existing embeddings for this content
            await self.embedding_service.delete_embeddings_for_content(
                content.content_id
            )

            # Process the updated content
            embedding_ids = await self.process_single_content(content)

            logger.info(f"Successfully updated content {content.content_id}")
            return embedding_ids

        except Exception as e:
            logger.error(f"Error updating content {content.content_id}: {str(e)}")
            raise ContentProcessingError(
                f"Failed to update content {content.content_id}: {str(e)}"
            )

    async def ingest_from_textbook_structure(self, textbook_structure: dict) -> dict:
        """Ingest content from a structured textbook format."""
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
                            content_id=str(uuid.uuid4()),
                            title=section_data.get("title", f"Section {section_id}"),
                            text=section_data.get("content", ""),
                            module=module_id,
                            chapter=chapter_id,
                            section=section_id,
                            page_numbers=section_data.get("page_numbers"),
                            metadata=section_data.get("metadata", {}),
                        )

                        contents.append(content)

            # Process all contents in batch
            results = await self.process_content_batch(contents)

            logger.info(
                f"Textbook ingestion completed: {len(results['successful'])} successful, {len(results['failed'])} failed"
            )
            return results

        except Exception as e:
            logger.error(f"Error ingesting from textbook structure: {str(e)}")
            raise ContentProcessingError(
                f"Failed to ingest textbook structure: {str(e)}"
            )

    async def reindex_content(self, content_id: str) -> bool:
        """Re-index a specific content piece (regenerate embeddings)."""
        try:
            # Get the existing content
            content = await self.db.get_content_by_id(content_id)
            if not content:
                raise ContentProcessingError(f"Content with ID {content_id} not found")

            # Update the content (which regenerates embeddings)
            await self.update_content(content)

            logger.info(f"Successfully reindexed content {content_id}")
            return True

        except Exception as e:
            logger.error(f"Error reindexing content {content_id}: {str(e)}")
            raise ContentProcessingError(
                f"Failed to reindex content {content_id}: {str(e)}"
            )

    async def reindex_all_content(self) -> dict:
        """Re-index all content in the system."""
        try:
            # Get all content IDs
            content_ids = await self.db.get_all_content_ids()

            # Get all content objects
            all_contents = []
            for content_id in content_ids:
                content = await self.db.get_content_by_id(content_id)
                if content:
                    all_contents.append(content)

            # Process all contents in batch
            results = await self.process_content_batch(all_contents)

            logger.info(
                f"Reindexing all content completed: {len(results['successful'])} successful, {len(results['failed'])} failed"
            )
            return results

        except Exception as e:
            logger.error(f"Error reindexing all content: {str(e)}")
            raise ContentProcessingError(f"Failed to reindex all content: {str(e)}")


# Singleton instance
content_processing_pipeline = ContentProcessingPipeline()
