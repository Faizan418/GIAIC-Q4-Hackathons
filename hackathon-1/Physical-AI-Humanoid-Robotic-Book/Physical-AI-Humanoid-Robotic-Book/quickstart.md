# Quickstart Guide for RAG Chatbot API

This guide will help you quickly set up and run the RAG Chatbot API for the Physical AI & Humanoid Robotics textbook.

## Prerequisites

- Python 3.12 or higher
- pip package manager
- Access to Cohere API
- Access to Qdrant vector database
- Access to Neon Postgres database

## Setup

### 1. Clone the Repository

```bash
git clone <your-repo-url>
cd Physical-AI-Humanoid-Robotic-Book
cd backend
```

### 2. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Set Up Environment Variables

Create a `.env` file in the `backend` directory with the following variables:

```bash
COHERE_API_KEY=your_cohere_api_key_here
NEON_POSTGRES_URL=your_neon_postgres_connection_string_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION=your_collection_name_here
OPENAI_API_KEY=your_openai_api_key_here
UVICORN_HOST=0.0.0.0
UVICORN_PORT=8000
```

## Running the Application

### 1. Start the API Server

```bash
python -m backend.main
```

Or with uvicorn directly:

```bash
uvicorn backend.main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Verify the Server is Running

Visit `http://localhost:8000/health` to check if the server is running properly.

You should see a response like:

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

## Using the API

### 1. Query the Textbook Content

Make a POST request to `/api/query`:

```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "context": "Introduction to Physical AI concepts",
    "user_id": "test_user_123"
  }'
```

### 2. Query Selected Text

Make a POST request to `/api/text-selection-query`:

```bash
curl -X POST http://localhost:8000/api/text-selection-query \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "Physical AI is an approach that emphasizes the importance of physics-based reasoning",
    "context": "Introduction section",
    "user_id": "test_user_123"
  }'
```

## Adding Content to the Knowledge Base

To add textbook content to the RAG system, you'll need to use the content processing pipeline. The system will automatically chunk the content, generate embeddings using Cohere, and store both in Qdrant (for similarity search) and Neon Postgres (for metadata).

## Testing

Run the tests to ensure everything is working correctly:

```bash
# Run unit tests
python -m pytest backend/tests/unit/

# Run integration tests
python -m pytest backend/tests/integration/

# Run contract tests
python -m pytest backend/tests/contract/
```

## API Endpoints

- `GET /health` - Health check for the entire API
- `POST /api/query` - Query the textbook content
- `POST /api/text-selection-query` - Query based on selected text
- `GET /api/query/health` - Health check for query endpoint
- `GET /api/text-selection/health` - Health check for text selection endpoint

## Troubleshooting

### Common Issues:

1. **Connection errors**: Verify your API keys and database URLs in the `.env` file
2. **Rate limit errors**: Check your Cohere and Qdrant usage limits
3. **Validation errors**: Ensure query text is not empty and under 1000 characters

### Logging:

Check the console output for detailed logs of API calls and any errors that occur.

## Next Steps

1. Add your textbook content to the knowledge base
2. Test different types of queries
3. Monitor performance and adjust as needed
4. Set up production deployment with appropriate security measures
