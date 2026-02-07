"""RAG (Retrieval-Augmented Generation) service for the chatbot."""

import logging
import uuid
from datetime import datetime
from typing import Any, Dict, Optional

import openai

from backend.config import settings
from backend.db.neon_postgres import db
from backend.embeddings.cohere_embed import cohere_service
from backend.models.query import Query, QueryRequest
from backend.models.response import QueryResponse, ResponseModel, SourceCitation
from backend.services.conversation_service import ConversationService
from backend.utils.exceptions import ExternalServiceError, RAGException
from backend.utils.logging import rag_logger
from backend.vectorstore.qdrant_client import qdrant_client

logger = logging.getLogger(__name__)


class RAGService:
    """Service class to handle RAG operations."""

    def __init__(self):
        # Set OpenAI API key
        openai.api_key = settings.openai_api_key
        self.max_retries = 3

    async def initialize(self):
        """Initialize the RAG service by connecting to databases."""
        try:
            await db.connect()
            await qdrant_client.initialize()
            logger.info("RAG service initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize RAG service: {e}")
            raise

    async def process_query(self, query_request: QueryRequest) -> QueryResponse:
        """Process a user query and return a response with citations."""
        start_time = datetime.utcnow()
        query_id = str(uuid.uuid4())
        rag_logger.log_query(query_id, query_request.query, query_request.user_id)

        try:
            # Get conversation service for tracking
            conversation_service = ConversationService()

            # Create or update conversation thread if provided
            conversation_thread = None
            if query_request.conversation_id:
                conversation_thread = (
                    await conversation_service.get_conversation_thread(
                        query_request.conversation_id
                    )
                )
                if not conversation_thread:
                    # Create new conversation thread if it doesn't exist
                    conversation_thread = await conversation_service.create_or_update_conversation_thread(
                        user_id=query_request.user_id,
                        title=f"Query: {query_request.query[:50]}{'...' if len(query_request.query) > 50 else ''}",
                        metadata={"created_from_query": True},
                    )
            else:
                # Create a new conversation thread if none provided
                conversation_thread = await conversation_service.create_or_update_conversation_thread(
                    user_id=query_request.user_id,
                    title=f"Query: {query_request.query[:50]}{'...' if len(query_request.query) > 50 else ''}",
                    metadata={"created_from_query": True},
                )

            # Store the user's query as a conversation message
            user_message = (
                await conversation_service.create_or_update_conversation_message(
                    conversation_id=conversation_thread.id,
                    role="user",
                    content=query_request.query,
                    context_used=None,
                    citations=None,
                )
            )

            # Generate embedding for the query
            query_embedding = cohere_service.generate_query_embedding(
                query_request.query
            )

            # Search for similar content in the vector store
            search_results = await qdrant_client.search_similar(
                query_embedding, top_k=5
            )

            # Prepare context from search results
            context_texts = [result["text_chunk"] for result in search_results]
            context = " ".join(context_texts)

            # Generate response using OpenAI
            response_text = await self._generate_response_with_context(
                query_request.query, context
            )

            # Create source citations
            source_citations = []
            for result in search_results:
                citation = SourceCitation(
                    citation_id=str(uuid.uuid4()),
                    content_id=result["content_id"],
                    title=f"Content {result['content_id'][:20]}...",  # Placeholder title
                    text_excerpt=(
                        result["text_chunk"][:200] + "..."
                        if len(result["text_chunk"]) > 200
                        else result["text_chunk"]
                    ),
                    module="unknown",  # Would come from content metadata
                    chapter="unknown",  # Would come from content metadata
                    section="unknown",  # Would come from content metadata
                    relevance_score=result["relevance_score"],
                )
                source_citations.append(citation)

            # Create response model
            response_id = str(uuid.uuid4())
            response_model = ResponseModel(
                response_id=response_id,
                query_id=query_id,
                answer_text=response_text,
                source_citations=source_citations,
                confidence_score=min(
                    1.0, len(context_texts) * 0.2
                ),  # Simple confidence calculation
                timestamp=datetime.utcnow(),
                query_text=query_request.query,
            )

            # Store the assistant's response as a conversation message
            assistant_message = (
                await conversation_service.create_or_update_conversation_message(
                    conversation_id=conversation_thread.id,
                    role="assistant",
                    content=response_text,
                    context_used={"context_chunks": len(context_texts)},
                    citations={"citation_count": len(source_citations)},
                )
            )

            # Calculate response time
            response_time = (datetime.utcnow() - start_time).total_seconds()

            # Track user analytics
            await conversation_service.create_or_update_user_analytics(
                user_id=query_request.user_id,
                session_id=query_request.conversation_id or conversation_thread.id,
                query=query_request.query,
                response_time=response_time,
                was_answered=True,
                used_selected_snippet=bool(
                    query_request.context
                ),  # If context was provided, it's from selected text
                satisfaction_score=None,  # Not provided by user
                was_accurate=None,  # Not provided by user
            )

            # Save to database
            await db.save_query(
                Query(
                    query_text=query_request.query,
                    context=query_request.context,
                    user_id=query_request.user_id,
                    timestamp=datetime.utcnow(),
                )
            )
            await db.save_response(response_model)

            rag_logger.log_response(
                response_id, query_id, response_model.confidence_score
            )

            # Return the response in the API format
            return QueryResponse(
                response_id=response_id,
                answer=response_text,
                source_citations=source_citations,
                confidence_score=response_model.confidence_score,
                query_text=query_request.query,
                timestamp=response_model.timestamp,
                conversation_id=conversation_thread.id,  # Include conversation ID in response
            )

        except Exception as e:
            rag_logger.log_error(type(e).__name__, str(e), query_id)
            if isinstance(e, RAGException):
                raise
            else:
                raise RAGException(f"Error processing query: {str(e)}")

    async def process_text_selection_query(
        self,
        selected_text: str,
        context: Optional[str] = None,
        user_id: Optional[str] = None,
        conversation_id: Optional[str] = None,
    ) -> QueryResponse:
        """Process a text selection query and return contextual information."""
        start_time = datetime.utcnow()
        query_id = str(uuid.uuid4())
        query_text = f"Explain more about: {selected_text[:100]}{'...' if len(selected_text) > 100 else ''}"

        rag_logger.log_query(query_id, query_text, user_id or "text_selection_user")

        try:
            # Get conversation service for tracking
            conversation_service = ConversationService()

            # Create or update conversation thread if provided
            conversation_thread = None
            if conversation_id:
                conversation_thread = (
                    await conversation_service.get_conversation_thread(conversation_id)
                )
                if not conversation_thread:
                    # Create new conversation thread if it doesn't exist
                    conversation_thread = await conversation_service.create_or_update_conversation_thread(
                        user_id=user_id,
                        title=f"Text Selection: {selected_text[:50]}{'...' if len(selected_text) > 50 else ''}",
                        metadata={"created_from_text_selection": True},
                    )
            else:
                # Create a new conversation thread if none provided
                conversation_thread = await conversation_service.create_or_update_conversation_thread(
                    user_id=user_id,
                    title=f"Text Selection: {selected_text[:50]}{'...' if len(selected_text) > 50 else ''}",
                    metadata={"created_from_text_selection": True},
                )

            # Store the user's selected text as a conversation message
            user_message = (
                await conversation_service.create_or_update_conversation_message(
                    conversation_id=conversation_thread.id,
                    role="user",
                    content=selected_text,
                    context_used={"selected_text": True},
                    citations=None,
                )
            )

            # Generate embedding for the selected text
            query_embedding = cohere_service.generate_query_embedding(selected_text)

            # Search for similar content in the vector store
            search_results = await qdrant_client.search_similar(
                query_embedding, top_k=5
            )

            # Prepare context from search results
            context_texts = [result["text_chunk"] for result in search_results]
            full_context = selected_text + " " + " ".join(context_texts)

            # Generate response using OpenAI
            response_text = await self._generate_response_with_context(
                f"Provide more information about: {selected_text}",
                " ".join(context_texts),
            )

            # Create source citations
            source_citations = []
            for result in search_results:
                citation = SourceCitation(
                    citation_id=str(uuid.uuid4()),
                    content_id=result["content_id"],
                    title=f"Content {result['content_id'][:20]}...",  # Placeholder title
                    text_excerpt=(
                        result["text_chunk"][:200] + "..."
                        if len(result["text_chunk"]) > 200
                        else result["text_chunk"]
                    ),
                    module="unknown",  # Would come from content metadata
                    chapter="unknown",  # Would come from content metadata
                    section="unknown",  # Would come from content metadata
                    relevance_score=result["relevance_score"],
                )
                source_citations.append(citation)

            # Create response model
            response_id = str(uuid.uuid4())
            response_model = ResponseModel(
                response_id=response_id,
                query_id=query_id,
                answer_text=response_text,
                source_citations=source_citations,
                confidence_score=min(
                    1.0, len(context_texts) * 0.2
                ),  # Simple confidence calculation
                timestamp=datetime.utcnow(),
                query_text=selected_text,
            )

            # Store the assistant's response as a conversation message
            assistant_message = (
                await conversation_service.create_or_update_conversation_message(
                    conversation_id=conversation_thread.id,
                    role="assistant",
                    content=response_text,
                    context_used={"context_chunks": len(context_texts)},
                    citations={"citation_count": len(source_citations)},
                )
            )

            # Calculate response time
            response_time = (datetime.utcnow() - start_time).total_seconds()

            # Track user analytics
            await conversation_service.create_or_update_user_analytics(
                user_id=user_id,
                session_id=conversation_id or conversation_thread.id,
                query=selected_text,
                response_time=response_time,
                was_answered=True,
                used_selected_snippet=True,  # This is a text selection query
                satisfaction_score=None,  # Not provided by user
                was_accurate=None,  # Not provided by user
            )

            # Save to database
            await db.save_query(
                Query(
                    query_text=selected_text,
                    context=context,
                    user_id=user_id or "text_selection_user",
                    timestamp=datetime.utcnow(),
                )
            )
            await db.save_response(response_model)

            rag_logger.log_response(
                response_id, query_id, response_model.confidence_score
            )

            # Return the response in the API format
            return QueryResponse(
                response_id=response_id,
                answer=response_text,
                source_citations=source_citations,
                confidence_score=response_model.confidence_score,
                query_text=selected_text,
                timestamp=response_model.timestamp,
                conversation_id=conversation_thread.id,  # Include conversation ID in response
            )

        except Exception as e:
            rag_logger.log_error(type(e).__name__, str(e), query_id)
            if isinstance(e, RAGException):
                raise
            else:
                raise RAGException(f"Error processing text selection query: {str(e)}")

    async def _generate_response_with_context(self, query: str, context: str) -> str:
        """Generate a response using OpenAI with the provided context."""
        try:
            # Set OpenAI API key from settings
            openai.api_key = settings.openai_api_key

            # Prepare the prompt with context
            system_message = f"""You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
            Use the following context to answer the user's question. If the context doesn't contain relevant information,
            acknowledge this and provide a helpful response based on general knowledge. Always maintain academic integrity."""

            user_message = f"Context: {context}\n\nQuestion: {query}"

            response = await openai.ChatCompletion.acreate(
                model=settings.openai_model,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_message},
                ],
                max_tokens=500,
                temperature=0.7,
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logger.error(f"Error generating response with OpenAI: {e}")
            raise ExternalServiceError(f"OpenAI service error: {str(e)}")

    async def add_content_to_knowledge_base(self, content_data: Dict[str, Any]):
        """Add content to the knowledge base by generating embeddings and storing them."""
        try:
            # Save content to database
            from datetime import datetime

            from backend.models.content import TextbookContent

            # Create a TextbookContent object from the content_data
            content_obj = TextbookContent(
                content_id=content_data.get("content_id", str(uuid.uuid4())),
                title=content_data.get("title", ""),
                text=content_data.get("text", ""),
                module=content_data.get("module", ""),
                chapter=content_data.get("chapter", ""),
                section=content_data.get("section", ""),
                page_numbers=content_data.get("page_numbers", ""),
                metadata=content_data.get("metadata", {}),
                created_at=content_data.get("created_at", datetime.utcnow()),
                updated_at=content_data.get("updated_at", datetime.utcnow()),
            )

            content_id = await db.insert_content(content_obj)

            # Generate embeddings for the content
            text = content_data.get("text", "")
            title = content_data.get("title", "")

            # Simple chunking strategy - split by paragraphs
            paragraphs = text.split("\n\n")
            chunks_to_embed = []

            for i, paragraph in enumerate(paragraphs):
                if len(paragraph.strip()) > 10:  # Only process non-empty paragraphs
                    chunk_data = {
                        "text": paragraph,
                        "content_id": content_id,
                        "chunk_index": i,
                        "title": title,
                        "module": content_data.get("module", ""),
                        "chapter": content_data.get("chapter", ""),
                        "section": content_data.get("section", ""),
                    }
                    chunks_to_embed.append(chunk_data)

            # Generate embeddings for all chunks
            chunks_with_embeddings = cohere_service.embed_text_chunks(chunks_to_embed)

            # Store embeddings in Qdrant
            for chunk in chunks_with_embeddings:
                point_id = await qdrant_client.store_embedding(
                    embedding_id=f"{content_id}_chunk_{chunk['chunk_index']}",  # Use proper embedding_id
                    embedding=chunk["embedding"],
                    content_id=chunk["content_id"],
                    text_chunk=chunk["text"],  # Pass the text chunk content
                    module=chunk["module"],
                    chapter=chunk["chapter"],
                    section=chunk["section"],
                    metadata={
                        "chunk_index": chunk["chunk_index"],
                        "title": chunk["title"],
                        "module": chunk["module"],
                        "chapter": chunk["chapter"],
                        "section": chunk["section"],
                    },
                )
                rag_logger.log_embedding_process(
                    f"{content_id}:{chunk['chunk_index']}", "stored"
                )

            logger.info(
                f"Added content {content_id} with {len(chunks_to_embed)} chunks to knowledge base"
            )

        except Exception as e:
            logger.error(f"Error adding content to knowledge base: {e}")
            raise RAGException(f"Error adding content: {str(e)}")


# Global instance
rag_service = RAGService()
