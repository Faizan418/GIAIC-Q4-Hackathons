# Quickstart: Book Intelligence Agent

## Overview
This guide will help you get the Book Intelligence Agent up and running quickly. The agent is designed to answer questions exclusively based on book content stored in a knowledge base, with special handling for user-selected text snippets.

## Prerequisites
- Python 3.12
- Docker (for containerized deployment)
- Qdrant Cloud account and API key
- Cohere API key
- Neon Postgres account and connection string

## Environment Setup

1. Create a `.env` file in the backend directory:
```bash
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_POSTGRES_URL=your_neon_postgres_connection_string
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Running the Agent

1. Start the backend server:
```bash
cd backend
python start_server.py
```

The server will start on `http://localhost:8000`

## Using the API

### Basic Query
Send a question about the book content:
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main theme of chapter 3?"
  }'
```

### Query with Selected Snippet (Prioritized Context)
Provide a specific text snippet to prioritize in the response:
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept in more detail",
    "selected_snippet": "The main theme of this chapter is the relationship between technology and society..."
  }'
```

### Creating a Conversation Thread
Start a new conversation:
```bash
curl -X POST http://localhost:8000/api/conversations \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Questions about Chapter 3 themes"
  }'
```

### Continuing a Conversation
Include the conversation ID in your query to maintain context:
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Can you elaborate on that?",
    "conversation_id": "your-conversation-uuid"
  }'
```

## Response Format
The API returns responses in the following format:
```json
{
  "response": "The main theme of chapter 3 is...",
  "citations": [
    {
      "chapter": "Chapter 3",
      "section": "3.2",
      "page": 45
    }
  ],
  "was_answered_from_book": true,
  "confidence_score": 0.85,
  "context_used": [
    {
      "id": "chunk-uuid",
      "text": "The main theme of this chapter is the relationship...",
      "score": 0.92
    }
  ]
}
```

## Key Features
- **Global Search**: Queries the entire book database when no snippet is provided
- **Context Prioritization**: Prioritizes user-selected text snippets when provided
- **Grounding Enforcement**: Ensures responses are based only on provided materials
- **Citation Capability**: References chapter/section information when available
- **Conversation Persistence**: Maintains conversation history in Neon Postgres
- **No Hallucination**: Will politely state when information is not available in the book
