# Quickstart Guide: Backend RAG Chatbot API

## Prerequisites

- Python 3.12+
- pip package manager
- Git
- Access to Cohere API (API key)
- Access to OpenAI API (API key)
- Qdrant Cloud account (API key and URL)
- Neon Postgres database (connection URL)

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install fastapi uvicorn cohere openai psycopg2-binary qdrant-client python-dotenv
```

### 4. Set Up Environment Variables
Create a `.env` file in the project root:
```env
COHERE_API_KEY=your_cohere_api_key_here
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_POSTGRES_URL=your_neon_postgres_connection_string
UVICORN_HOST=0.0.0.0
UVICORN_PORT=8000
```

## Running the Application

### 1. Start the Server
```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Verify Installation
Open your browser or use curl to check the health endpoint:
```bash
curl http://localhost:8000/health
```

You should receive a response similar to:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-16T10:00:00Z",
  "dependencies": {
    "cohere_api": "connected",
    "qdrant": "connected",
    "postgres": "connected"
  }
}
```

## Testing the API

### Query Endpoint
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "context": "I want to understand the basic concept"
  }'
```

### Text Selection Query Endpoint
```bash
curl -X POST http://localhost:8000/text-selection-query \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "Physical AI is a new approach to robotics",
    "context": "Explain this concept in more detail"
  }'
```

## Initial Content Setup

To initialize the RAG system with textbook content:

1. Prepare your textbook content in the expected format
2. Run the embedding script to generate and store embeddings:
```bash
python -m embeddings.cohere_embed --input-path /path/to/textbook/content
```

This will:
- Process the textbook content
- Generate Cohere embeddings for each content chunk
- Store embeddings in Qdrant
- Store metadata in Neon Postgres

## Configuration

### Environment Variables
- `UVICORN_HOST`: Host for the Uvicorn server (default: 0.0.0.0)
- `UVICORN_PORT`: Port for the Uvicorn server (default: 8000)
- `COHERE_MODEL`: Cohere model to use for embeddings (default: embed-multilingual-v2.0)
- `QDRANT_COLLECTION`: Name of the Qdrant collection (default: textbook_embeddings)
- `MAX_CONCURRENT_REQUESTS`: Maximum concurrent requests (default: 100)

### Performance Tuning
- Adjust the number of Uvicorn workers based on your server capacity
- Configure connection pooling for database and vector store connections
- Set up response caching for frequently asked queries

## Troubleshooting

### Common Issues

1. **API Connection Errors**: Verify all API keys and connection URLs are correct
2. **Rate Limiting**: Check your API usage limits with Cohere and OpenAI
3. **Performance Issues**: Monitor resource usage and adjust server configuration
4. **Embedding Generation**: Large content sets may take time to process initially

### Health Checks
Use the `/health` endpoint to verify all services are connected and operational.
