# API Contract: Backend RAG Chatbot

## Query Endpoint

### POST /query

**Description**: Accepts user questions and returns relevant responses based on textbook content

**Request**:
```json
{
  "query": "string (required, 1-1000 characters)",
  "context": "string (optional, up to 2000 characters)",
  "user_id": "string (optional)"
}
```

**Response (Success - 200 OK)**:
```json
{
  "response_id": "string",
  "answer": "string",
  "source_citations": [
    {
      "content_id": "string",
      "title": "string",
      "text_excerpt": "string",
      "module": "string",
      "chapter": "string",
      "section": "string",
      "relevance_score": "number (0-1)"
    }
  ],
  "confidence_score": "number (0-1)",
  "query_text": "string",
  "timestamp": "ISO 8601 datetime"
}
```

**Response (Bad Request - 400)**:
```json
{
  "error": "string",
  "details": "string"
}
```

**Response (Service Unavailable - 503)**:
```json
{
  "error": "External service unavailable",
  "details": "Cohere API or OpenAI API temporarily unavailable"
}
```

## Text Selection Query Endpoint

### POST /text-selection-query

**Description**: Accepts selected text from the textbook and returns contextual information

**Request**:
```json
{
  "selected_text": "string (required, 1-1000 characters)",
  "context": "string (optional, up to 2000 characters)",
  "user_id": "string (optional)"
}
```

**Response (Success - 200 OK)**:
```json
{
  "response_id": "string",
  "answer": "string",
  "source_citations": [
    {
      "content_id": "string",
      "title": "string",
      "text_excerpt": "string",
      "module": "string",
      "chapter": "string",
      "section": "string",
      "relevance_score": "number (0-1)"
    }
  ],
  "confidence_score": "number (0-1)",
  "query_text": "string",
  "timestamp": "ISO 8601 datetime"
}
```

**Response (Bad Request - 400)**:
```json
{
  "error": "string",
  "details": "string"
}
```

## Health Check Endpoint

### GET /health

**Description**: Returns the health status of the service

**Response (Success - 200 OK)**:
```json
{
  "status": "healthy",
  "timestamp": "ISO 8601 datetime",
  "dependencies": {
    "cohere_api": "status",
    "qdrant": "status",
    "postgres": "status"
  }
}
```

## Error Response Format

All error responses follow this format:
```json
{
  "error": "string",
  "message": "string",
  "timestamp": "ISO 8601 datetime",
  "request_id": "string (if available)"
}
```

## Common Headers

- **Content-Type**: application/json
- **Authorization**: Bearer {API_KEY} (for authenticated endpoints)
- **X-Request-ID**: string (for request tracking)

## Rate Limiting

- Maximum 100 requests per minute per API key
- Exceeding limits returns 429 Too Many Requests with retry-after header
