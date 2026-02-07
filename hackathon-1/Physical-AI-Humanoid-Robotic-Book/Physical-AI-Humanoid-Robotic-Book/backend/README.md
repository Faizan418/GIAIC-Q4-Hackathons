# RAG Chatbot API Backend

This is the backend implementation for the RAG (Retrieval-Augmented Generation) Chatbot API for the Physical AI & Humanoid Robotics textbook project.

## Overview

The RAG Chatbot API provides intelligent querying capabilities for the Physical AI & Humanoid Robotics textbook content. It uses advanced AI techniques including vector embeddings and semantic search to provide accurate, cited responses to user queries.

## Architecture

### Tech Stack
- **Framework**: FastAPI + Uvicorn
- **Embeddings**: Cohere API
- **Vector Storage**: Qdrant
- **Metadata Storage**: Neon Serverless Postgres
- **Language**: Python 3.12

### Core Components
- **Models**: Pydantic models for request/response validation
- **API**: FastAPI endpoints for querying and text selection
- **Services**: Business logic for RAG orchestration
- **DB**: Neon Postgres integration
- **Vectorstore**: Qdrant vector database integration
- **Embeddings**: Cohere embedding service
- **Utils**: Error handling and logging utilities

## Features

### Query Functionality
- Submit queries about textbook content
- Receive relevant, cited responses
- Academic integrity maintained through citations

### Text Selection Queries
- Submit selected text from the textbook
- Receive contextual information expanding on selected content

### Content Management
- Process textbook content to generate embeddings
- Store content in Neon Postgres
- Store embeddings in Qdrant for similarity search

## API Endpoints

### Query Endpoint
- **POST** `/api/query`
- Accepts a query request with text and optional context
- Returns a response with answer and source citations

### Text Selection Query Endpoint
- **POST** `/api/text-selection-query`
- Accepts selected text and optional context
- Returns contextual information about the selected text

### Health Check Endpoints
- **GET** `/health`
- **GET** `/api/query/health`
- **GET** `/api/text-selection/health`

## Environment Variables

Create a `.env` file in the backend directory with the following variables:

```bash
COHERE_API_KEY=your_cohere_api_key
NEON_POSTGRES_URL=your_neon_postgres_connection_string
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION=your_collection_name
OPENAI_API_KEY=your_openai_api_key
UVICORN_HOST=0.0.0.0
UVICORN_PORT=8000
```

## Running the Application

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables in `.env`

3. Run the application:
```bash
python -m backend.main
```

Or with uvicorn directly:
```bash
uvicorn backend.main:app --reload --host 0.0.0.0 --port 8000
```

## Development

### Dependencies
- Python 3.12+
- FastAPI
- Uvicorn
- Cohere
- OpenAI
- psycopg2-binary
- qdrant-client
- python-dotenv
- asyncpg
- pydantic

### Code Formatting
- Black for code formatting
- Flake8 for linting
- MyPy for type checking

## Testing

Tests are located in the `backend/tests/` directory with the following structure:
- `contract/` - API contract tests
- `integration/` - Integration tests
- `unit/` - Unit tests

## Project Structure

```
backend/
├── main.py                 # FastAPI application entry point
├── config.py              # Configuration management
├── requirements.txt       # Dependencies
├── .env                  # Environment variables (not committed)
├── .gitignore            # Git ignore patterns
├── models/               # Pydantic data models
│   ├── query.py          # Query models
│   ├── response.py       # Response models
│   ├── content.py        # Textbook content model
│   └── embedding.py      # Embedding model
├── api/                  # API route definitions
│   ├── query.py          # Query endpoint
│   └── text_selection.py # Text selection endpoint
├── services/             # Business logic
│   ├── rag_service.py    # Core RAG orchestration
│   ├── embedding_service.py # Embedding operations
│   ├── content_processing_pipeline.py # Content ingestion
│   └── content_indexing_workflow.py # Indexing operations
├── db/                   # Database operations
│   └── neon_postgres.py  # Neon Postgres integration
├── vectorstore/          # Vector database
│   └── qdrant_client.py  # Qdrant integration
├── embeddings/           # Embedding operations
│   └── cohere_embed.py   # Cohere service
├── utils/                # Utilities
│   ├── exceptions.py     # Custom exceptions
│   └── logging.py        # Logging utilities
└── tests/                # Test files
    ├── contract/         # Contract tests
    ├── integration/      # Integration tests
    └── unit/             # Unit tests
```

## Error Handling

The application implements comprehensive error handling with:
- Validation errors for malformed requests
- Service-specific exceptions
- Proper HTTP status codes
- Detailed error messages for debugging

## Security

- Input validation with Pydantic models
- Secure API key management
- Rate limiting considerations
- Error message sanitization

## Performance

- Async/await patterns for high concurrency
- Connection pooling for database efficiency
- Efficient vector search with Qdrant
- Optimized embedding generation

## Logging

The application includes comprehensive logging for:
- API calls with timing
- Query and response tracking
- Embedding generation and storage
- Error tracking and debugging

## Integration with Frontend

This backend is designed to integrate with the Docusaurus frontend for the AI-Native textbook, providing seamless querying capabilities within the educational content.

## Deployment

For production deployment:
1. Ensure all environment variables are properly configured
2. Set up Neon Postgres and Qdrant with appropriate scaling
3. Configure load balancing if needed
4. Set up monitoring and alerting
