# Text Embedding Handler

A Python script that handles text embedding using Cohere SDK and stores embeddings in Qdrant with upsert functionality.

## Features

- Text chunking with configurable size and overlap
- Embedding generation using Cohere's `embed-english-v3.0` model
- Proper input_type handling (`search_document` for storage, `search_query` for user questions)
- Upsert logic for Qdrant collection to prevent duplicate entries
- Comprehensive error handling
- Environment variable management for API keys
- Support for metadata storage with embeddings

## Requirements

- Python 3.8+
- Cohere API key
- Qdrant instance (local or cloud)

## Installation

1. Install required packages:
   ```bash
   pip install -r scripts/embedding_requirements.txt
   ```

2. Set up environment variables:
   ```bash
   export COHERE_API_KEY="your-cohere-api-key"
   export QDRANT_URL="your-qdrant-url"  # Optional, defaults to http://localhost:6333
   export QDRANT_API_KEY="your-qdrant-api-key"  # Optional, for cloud instances
   ```

## Usage

### Command Line

Run the script directly to see example usage:
```bash
python scripts/text_embedding_handler.py
```

### As a Module

```python
from scripts.text_embedding_handler import TextEmbeddingHandler

# Initialize handler
handler = TextEmbeddingHandler(
    cohere_api_key="your-key",  # or set via env var
    qdrant_url="your-url",      # or set via env var
    qdrant_api_key="your-key"   # or set via env var
)

# Chunk and embed text
text = "Your long text here..."
chunks = handler.chunk_text(text, chunk_size=1000, overlap=100)

# Upsert embeddings with metadata
metadata_list = [{"source": "book", "chapter": "1"}] * len(chunks)
handler.upsert_embeddings(
    texts=chunks,
    metadatas=metadata_list,
    input_type="search_document"  # For document storage
)

# Search for similar content
results = handler.search_similar("your query", limit=5)
```

## API Key Configuration

The script looks for API keys in this order:
1. Parameters passed to the constructor
2. Environment variables:
   - `COHERE_API_KEY`
   - `QDRANT_URL` (defaults to `http://localhost:6333`)
   - `QDRANT_API_KEY` (optional, for cloud instances)

## Input Types

- `search_document`: Use when storing document chunks for retrieval
- `search_query`: Use when embedding user queries for search
- Other valid types: `classification`, `clustering`

## Methods

### `chunk_text(text, chunk_size=1000, overlap=100)`
Splits text into overlapping chunks.

### `embed_texts(texts, input_type="search_document")`
Generates embeddings for a list of texts.

### `upsert_embeddings(texts, metadatas=None, ids=None, input_type="search_document")`
Upserts text embeddings to Qdrant collection.

### `search_similar(query, limit=10, filters=None)`
Searches for similar embeddings in the collection.

### `get_embedding_count()`
Returns the total number of embeddings in the collection.

## Environment Variables

Create a `.env` file in the project root:
```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
```

The script will automatically load these variables.

## Error Handling

The script includes comprehensive error handling for:
- API connection issues
- Invalid input parameters
- Qdrant collection operations
- Embedding generation failures

## License

This script is provided as part of the Physical AI & Humanoid Robotics project.
