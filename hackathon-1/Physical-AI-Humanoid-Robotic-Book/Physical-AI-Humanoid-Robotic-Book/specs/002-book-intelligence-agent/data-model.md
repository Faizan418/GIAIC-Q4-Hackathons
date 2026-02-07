# Data Model: Book Intelligence Agent

## Entities

### ConversationThread
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to user, optional for anonymous sessions)
- **created_at**: DateTime (Timestamp when conversation started)
- **updated_at**: DateTime (Timestamp when conversation was last updated)
- **title**: String (Brief summary of the conversation topic)
- **metadata**: JSON (Additional conversation metadata)

### ConversationMessage
- **id**: UUID (Primary Key)
- **conversation_id**: UUID (Foreign Key to ConversationThread)
- **role**: String (Either "user" or "assistant")
- **content**: Text (The message content)
- **timestamp**: DateTime (When the message was created)
- **context_used**: JSON (The context provided to the LLM for this response)
- **citations**: JSON (List of citations used in the response)

### UserAnalytics
- **id**: UUID (Primary Key)
- **user_id**: UUID (Unique identifier for the user, can be anonymous)
- **session_id**: UUID (Unique identifier for the current session)
- **query**: Text (The user's original query)
- **response_time**: Float (Time taken to generate the response in seconds)
- **satisfaction_score**: Integer (Optional user rating 1-5)
- **timestamp**: DateTime (When the interaction occurred)
- **was_answered**: Boolean (Whether the system could answer the question)
- **was_accurate**: Boolean (Whether the user indicated the answer was accurate)
- **used_selected_snippet**: Boolean (Whether the user provided a selected text snippet)

### BookContentChunk
- **id**: UUID (Primary Key, same as Qdrant point ID for consistency)
- **book_id**: String (Identifier for the book)
- **chunk_text**: Text (The actual text content of the chunk)
- **embedding_vector**: JSON (Vector representation of the content)
- **metadata**: JSON (Additional metadata like chapter, section, page number)
- **created_at**: DateTime (When this chunk was indexed)
- **updated_at**: DateTime (When this chunk was last updated)

### QueryRequest
- **query**: Text (The user's question)
- **selected_snippet**: Text (Optional user-selected text to prioritize)
- **conversation_id**: UUID (Optional ID of the conversation thread)
- **user_id**: UUID (Optional ID of the user)

### QueryResponse
- **response**: Text (The agent's answer to the query)
- **citations**: Array[Object] (List of citations with chapter/section info)
- **was_answered_from_book**: Boolean (Whether the information came from the book)
- **confidence_score**: Float (Confidence level of the response)
- **context_used**: Array[Object] (The context chunks that informed the response)
- **conversation_id**: UUID (ID of the conversation thread)
