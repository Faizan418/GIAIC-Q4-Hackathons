"""Database connection for Neon Postgres in the RAG Chatbot API."""

import logging
from datetime import datetime
from typing import Any, Dict, List, Optional

import asyncpg

from backend.config import get_settings
from backend.models.content import TextbookContent
from backend.models.embedding import EmbeddingModel
from backend.models.query import Query
from backend.models.response import ResponseModel

logger = logging.getLogger(__name__)


class NeonPostgresDB:
    """Class to handle Neon Postgres database operations."""

    def __init__(self):
        settings = get_settings()
        self.connection_string = settings.neon_postgres_url
        self.pool: Optional[asyncpg.Pool] = None

    async def connect(self):
        """Establish connection to the database."""
        try:
            self.pool = await asyncpg.create_pool(
                self.connection_string, min_size=5, max_size=20, command_timeout=60
            )
            logger.info("Connected to Neon Postgres database")
        except Exception as e:
            logger.error(f"Failed to connect to Neon Postgres: {e}")
            raise

    async def disconnect(self):
        """Close the database connection."""
        if self.pool:
            await self.pool.close()
            logger.info("Disconnected from Neon Postgres database")

    async def save_query(self, query: Query) -> str:
        """Save a query to the database."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            # Create table if it doesn't exist
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS queries (
                    id SERIAL PRIMARY KEY,
                    query_text TEXT NOT NULL,
                    context TEXT,
                    user_id VARCHAR(255),
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """
            )

            query_id = await conn.fetchval(
                """
                INSERT INTO queries (query_text, context, user_id, timestamp)
                VALUES ($1, $2, $3, $4)
                RETURNING id
                """,
                query.query_text,
                query.context,
                query.user_id,
                query.timestamp,
            )
            return str(query_id)

    async def save_response(self, response: ResponseModel) -> str:
        """Save a response to the database."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            # Create table if it doesn't exist
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS responses (
                    id SERIAL PRIMARY KEY,
                    response_id VARCHAR(255) UNIQUE NOT NULL,
                    query_id VARCHAR(255) NOT NULL,
                    answer_text TEXT NOT NULL,
                    confidence_score FLOAT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    query_text TEXT
                )
            """
            )

            # Create table for citations if it doesn't exist
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS citations (
                    id SERIAL PRIMARY KEY,
                    response_id VARCHAR(255) NOT NULL,
                    content_id VARCHAR(255) NOT NULL,
                    title TEXT,
                    text_excerpt TEXT,
                    module VARCHAR(255),
                    chapter VARCHAR(255),
                    section VARCHAR(255),
                    relevance_score FLOAT
                )
            """
            )

            await conn.execute(
                """
                INSERT INTO responses (response_id, query_id, answer_text, confidence_score, timestamp, query_text)
                VALUES ($1, $2, $3, $4, $5, $6)
                """,
                response.response_id,
                response.query_id,
                response.answer_text,
                response.confidence_score,
                response.timestamp,
                response.query_text,
            )

            # Save citations
            for citation in response.source_citations:
                await conn.execute(
                    """
                    INSERT INTO citations (response_id, content_id, title, text_excerpt, module, chapter, section, relevance_score)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
                    """,
                    response.response_id,
                    citation.content_id,
                    citation.title,
                    citation.text_excerpt,
                    citation.module,
                    citation.chapter,
                    citation.section,
                    citation.relevance_score,
                )

            return response.response_id

    async def get_content_by_id(self, content_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve content by ID from the database."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            # This is a placeholder - actual implementation would depend on the content table structure
            row = await conn.fetchrow(
                """
                SELECT content_id, title, text, module, chapter, section, page_numbers, metadata, created_at, updated_at
                FROM textbook_content
                WHERE content_id = $1
                """,
                content_id,
            )

            if row:
                return {
                    "content_id": row["content_id"],
                    "title": row["title"],
                    "text": row["text"],
                    "module": row["module"],
                    "chapter": row["chapter"],
                    "section": row["section"],
                    "page_numbers": row["page_numbers"],
                    "metadata": row["metadata"],
                    "created_at": row["created_at"],
                    "updated_at": row["updated_at"],
                }
            return None

    async def insert_content(self, content: "TextbookContent") -> str:
        """Insert textbook content to the database."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            # Create table if it doesn't exist
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS textbook_content (
                    id SERIAL PRIMARY KEY,
                    content_id VARCHAR(255) UNIQUE NOT NULL,
                    title TEXT NOT NULL,
                    text TEXT NOT NULL,
                    module VARCHAR(255),
                    chapter VARCHAR(255),
                    section VARCHAR(255),
                    page_numbers VARCHAR(255),
                    metadata JSONB,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """
            )

            content_id = await conn.fetchval(
                """
                INSERT INTO textbook_content (content_id, title, text, module, chapter, section, page_numbers, metadata, created_at, updated_at)
                VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
                ON CONFLICT (content_id) DO UPDATE SET
                    title = EXCLUDED.title,
                    text = EXCLUDED.text,
                    module = EXCLUDED.module,
                    chapter = EXCLUDED.chapter,
                    section = EXCLUDED.section,
                    page_numbers = EXCLUDED.page_numbers,
                    metadata = EXCLUDED.metadata,
                    updated_at = EXCLUDED.updated_at
                RETURNING content_id
                """,
                content.content_id,
                content.title,
                content.text,
                content.module,
                content.chapter,
                content.section,
                content.page_numbers,
                content.metadata,
                content.created_at,
                content.updated_at,
            )
            return content_id

    async def update_content(self, content: "TextbookContent") -> bool:
        """Update existing textbook content in the database."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            result = await conn.execute(
                """
                UPDATE textbook_content
                SET title = $1, text = $2, module = $3, chapter = $4, section = $5,
                    page_numbers = $6, metadata = $7, updated_at = $8
                WHERE content_id = $9
                """,
                content.title,
                content.text,
                content.module,
                content.chapter,
                content.section,
                content.page_numbers,
                content.metadata,
                content.updated_at,
                content.content_id,
            )
            return result != "UPDATE 0"

    async def delete_content(self, content_id: str) -> bool:
        """Delete textbook content from the database."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            result = await conn.execute(
                "DELETE FROM textbook_content WHERE content_id = $1", content_id
            )
            return result != "DELETE 0"

    async def get_all_content_ids(self) -> List[str]:
        """Get all content IDs from the database."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            rows = await conn.fetch("SELECT content_id FROM textbook_content")
            return [row["content_id"] for row in rows]

    async def search_content(
        self,
        query: str,
        module: Optional[str] = None,
        chapter: Optional[str] = None,
        section: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """Search for content based on text and filters."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            # Build the query with optional filters
            where_conditions = [
                "text ILIKE $"
                + str(
                    len([module, chapter, section])
                    - [module, chapter, section].count(None)
                    + 1
                )
            ]
            params = [f"%{query}%"]
            param_index = 2

            if module:
                where_conditions.append(f"module = ${param_index}")
                params.append(module)
                param_index += 1
            if chapter:
                where_conditions.append(f"chapter = ${param_index}")
                params.append(chapter)
                param_index += 1
            if section:
                where_conditions.append(f"section = ${param_index}")
                params.append(section)
                param_index += 1

            where_clause = " AND ".join(where_conditions)
            sql = f"""
                SELECT content_id, title, text, module, chapter, section, page_numbers, metadata, created_at, updated_at
                FROM textbook_content
                WHERE {where_clause}
                ORDER BY created_at DESC
            """

            rows = await conn.fetch(sql, *params)
            return [
                {
                    "content_id": row["content_id"],
                    "title": row["title"],
                    "text": row["text"],
                    "module": row["module"],
                    "chapter": row["chapter"],
                    "section": row["section"],
                    "page_numbers": row["page_numbers"],
                    "metadata": row["metadata"],
                    "created_at": row["created_at"],
                    "updated_at": row["updated_at"],
                }
                for row in rows
            ]

    async def insert_embedding(self, embedding: "EmbeddingModel") -> str:
        """Insert embedding record to the database."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            # Create table if it doesn't exist
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS embeddings (
                    id SERIAL PRIMARY KEY,
                    embedding_id VARCHAR(255) UNIQUE NOT NULL,
                    content_id VARCHAR(255) NOT NULL,
                    embedding FLOAT[], -- Array of floats for the embedding vector
                    model_name VARCHAR(255) NOT NULL,
                    module VARCHAR(255),
                    chapter VARCHAR(255),
                    section VARCHAR(255),
                    metadata JSONB,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """
            )

            # Note: Postgres doesn't directly support float arrays in asyncpg, so we'll store as JSON
            embedding_id = await conn.fetchval(
                """
                INSERT INTO embeddings (embedding_id, content_id, embedding, model_name, module, chapter, section, metadata, created_at, updated_at)
                VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
                ON CONFLICT (embedding_id) DO UPDATE SET
                    content_id = EXCLUDED.content_id,
                    embedding = EXCLUDED.embedding,
                    model_name = EXCLUDED.model_name,
                    module = EXCLUDED.module,
                    chapter = EXCLUDED.chapter,
                    section = EXCLUDED.section,
                    metadata = EXCLUDED.metadata,
                    updated_at = EXCLUDED.updated_at
                RETURNING embedding_id
                """,
                embedding.embedding_id,
                embedding.content_id,
                embedding.embedding,  # This will be stored as JSON since Postgres float[] is complex with asyncpg
                embedding.model_name,
                embedding.module,
                embedding.chapter,
                embedding.section,
                embedding.metadata,
                embedding.created_at,
                embedding.updated_at,
            )
            return embedding_id

    async def get_embedding_by_id(self, embedding_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve embedding by ID from the database."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT embedding_id, content_id, embedding, model_name, module, chapter, section, metadata, created_at, updated_at
                FROM embeddings
                WHERE embedding_id = $1
                """,
                embedding_id,
            )

            if row:
                return {
                    "embedding_id": row["embedding_id"],
                    "content_id": row["content_id"],
                    "embedding": row["embedding"],
                    "model_name": row["model_name"],
                    "module": row["module"],
                    "chapter": row["chapter"],
                    "section": row["section"],
                    "metadata": row["metadata"],
                    "created_at": row["created_at"],
                    "updated_at": row["updated_at"],
                }
            return None

    async def get_embedding_ids_by_content_id(self, content_id: str) -> List[str]:
        """Get all embedding IDs associated with a content ID."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                "SELECT embedding_id FROM embeddings WHERE content_id = $1", content_id
            )
            return [row["embedding_id"] for row in rows]

    async def delete_embeddings_by_content_id(self, content_id: str) -> int:
        """Delete all embeddings associated with a content ID."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            result = await conn.execute(
                "DELETE FROM embeddings WHERE content_id = $1", content_id
            )
            # Extract the number of deleted rows
            deleted_count = int(result.split(" ")[1]) if "DELETE " in result else 0
            return deleted_count

    async def insert_conversation_thread(
        self,
        thread_id: str,
        user_id: Optional[str],
        title: str,
        metadata: Dict[str, Any],
        created_at: datetime,
        updated_at: datetime,
    ) -> str:
        """Insert or update a conversation thread with upsert logic."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            # Create conversation_threads table if it doesn't exist
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS conversation_threads (
                    id VARCHAR(255) PRIMARY KEY,
                    user_id VARCHAR(255),
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    title TEXT NOT NULL,
                    metadata JSONB
                )
                """
            )

            # Insert or update the conversation thread
            result_id = await conn.fetchval(
                """
                INSERT INTO conversation_threads (id, user_id, created_at, updated_at, title, metadata)
                VALUES ($1, $2, $3, $4, $5, $6)
                ON CONFLICT (id) DO UPDATE SET
                    user_id = EXCLUDED.user_id,
                    updated_at = EXCLUDED.updated_at,
                    title = EXCLUDED.title,
                    metadata = EXCLUDED.metadata
                RETURNING id
                """,
                thread_id,
                user_id,
                created_at,
                updated_at,
                title,
                metadata,
            )
            return result_id

    async def get_conversation_thread(self, thread_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve a conversation thread by ID."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT id, user_id, created_at, updated_at, title, metadata
                FROM conversation_threads
                WHERE id = $1
                """,
                thread_id,
            )

            if row:
                return {
                    "id": row["id"],
                    "user_id": row["user_id"],
                    "created_at": row["created_at"],
                    "updated_at": row["updated_at"],
                    "title": row["title"],
                    "metadata": row["metadata"],
                }
            return None

    async def insert_conversation_message(
        self,
        message_id: str,
        conversation_id: str,
        role: str,
        content: str,
        timestamp: datetime,
        context_used: Dict[str, Any],
        citations: Dict[str, Any],
    ) -> str:
        """Insert or update a conversation message with upsert logic."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            # Create conversation_messages table if it doesn't exist
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS conversation_messages (
                    id VARCHAR(255) PRIMARY KEY,
                    conversation_id VARCHAR(255) NOT NULL,
                    role VARCHAR(20) NOT NULL,
                    content TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    context_used JSONB,
                    citations JSONB,
                    FOREIGN KEY (conversation_id) REFERENCES conversation_threads(id)
                )
                """
            )

            # Insert or update the conversation message
            result_id = await conn.fetchval(
                """
                INSERT INTO conversation_messages (id, conversation_id, role, content, timestamp, context_used, citations)
                VALUES ($1, $2, $3, $4, $5, $6, $7)
                ON CONFLICT (id) DO UPDATE SET
                    conversation_id = EXCLUDED.conversation_id,
                    role = EXCLUDED.role,
                    content = EXCLUDED.content,
                    timestamp = EXCLUDED.timestamp,
                    context_used = EXCLUDED.context_used,
                    citations = EXCLUDED.citations
                RETURNING id
                """,
                message_id,
                conversation_id,
                role,
                content,
                timestamp,
                context_used,
                citations,
            )
            return result_id

    async def get_conversation_messages(
        self, conversation_id: str
    ) -> List[Dict[str, Any]]:
        """Retrieve all messages for a conversation."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT id, conversation_id, role, content, timestamp, context_used, citations
                FROM conversation_messages
                WHERE conversation_id = $1
                ORDER BY timestamp ASC
                """,
                conversation_id,
            )

            messages = []
            for row in rows:
                message = {
                    "id": row["id"],
                    "conversation_id": row["conversation_id"],
                    "role": row["role"],
                    "content": row["content"],
                    "timestamp": row["timestamp"],
                    "context_used": row["context_used"],
                    "citations": row["citations"],
                }
                messages.append(message)

            return messages

    async def insert_user_analytics(
        self,
        analytics_id: str,
        user_id: Optional[str],
        session_id: str,
        query: str,
        response_time: float,
        satisfaction_score: Optional[int],
        timestamp: datetime,
        was_answered: bool,
        was_accurate: Optional[bool],
        used_selected_snippet: bool,
    ) -> str:
        """Insert or update a user analytics record with upsert logic."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            # Create user_analytics table if it doesn't exist
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS user_analytics (
                    id VARCHAR(255) PRIMARY KEY,
                    user_id VARCHAR(255),
                    session_id VARCHAR(255) NOT NULL,
                    query TEXT NOT NULL,
                    response_time FLOAT NOT NULL,
                    satisfaction_score INTEGER,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    was_answered BOOLEAN NOT NULL,
                    was_accurate BOOLEAN,
                    used_selected_snippet BOOLEAN NOT NULL
                )
                """
            )

            # Insert or update the user analytics record
            result_id = await conn.fetchval(
                """
                INSERT INTO user_analytics (id, user_id, session_id, query, response_time, satisfaction_score, timestamp, was_answered, was_accurate, used_selected_snippet)
                VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
                ON CONFLICT (id) DO UPDATE SET
                    user_id = EXCLUDED.user_id,
                    session_id = EXCLUDED.session_id,
                    query = EXCLUDED.query,
                    response_time = EXCLUDED.response_time,
                    satisfaction_score = EXCLUDED.satisfaction_score,
                    timestamp = EXCLUDED.timestamp,
                    was_answered = EXCLUDED.was_answered,
                    was_accurate = EXCLUDED.was_accurate,
                    used_selected_snippet = EXCLUDED.used_selected_snippet
                RETURNING id
                """,
                analytics_id,
                user_id,
                session_id,
                query,
                response_time,
                satisfaction_score,
                timestamp,
                was_answered,
                was_accurate,
                used_selected_snippet,
            )
            return result_id

    async def get_user_analytics_by_user_id(self, user_id: str) -> List[Dict[str, Any]]:
        """Retrieve user analytics by user ID."""
        if not self.pool:
            raise Exception("Database not connected")

        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT id, user_id, session_id, query, response_time, satisfaction_score, timestamp, was_answered, was_accurate, used_selected_snippet
                FROM user_analytics
                WHERE user_id = $1
                ORDER BY timestamp DESC
                """,
                user_id,
            )

            analytics_list = []
            for row in rows:
                analytics = {
                    "id": row["id"],
                    "user_id": row["user_id"],
                    "session_id": row["session_id"],
                    "query": row["query"],
                    "response_time": row["response_time"],
                    "satisfaction_score": row["satisfaction_score"],
                    "timestamp": row["timestamp"],
                    "was_answered": row["was_answered"],
                    "was_accurate": row["was_accurate"],
                    "used_selected_snippet": row["used_selected_snippet"],
                }
                analytics_list.append(analytics)

            return analytics_list


# Global instance - will be created when accessed for the first time
class _DatabaseProvider:
    def __init__(self):
        self._instance = None

    def __getattr__(self, name):
        if self._instance is None:
            self._instance = NeonPostgresDB()
        return getattr(self._instance, name)

    def __setattr__(self, name, value):
        if name.startswith("_"):
            super().__setattr__(name, value)
        else:
            if self._instance is None:
                self._instance = NeonPostgresDB()
            setattr(self._instance, name, value)


db = _DatabaseProvider()
