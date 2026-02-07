# Feature Specification: Backend RAG Chatbot API

**Feature Branch**: `001-backend-rag-chatbot`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "used UV package require backend

## Module: Backend RAG Chatbot

**Project:** Physical AI & Humanoid Robotics – AI-Native Textbook
**Status:** Production-Ready

---

### 1. Purpose
Develop the **backend API** for the RAG chatbot that:

- Runs on **FastAPI + Uvicorn**
- Handles `/query` and `/text-selection-query` endpoints
- Processes book content and queries using **Cohere embeddings**
- Stores/retrieves data from Neon Postgres + Qdrant

---

### 2. Functional Requirements
- Endpoint `/query` for general user queries
- Endpoint `/text-selection-query` for selected text queries
- Cohere embedding generation for book content
- Vector search in Qdrant
- RAG orchestration with OpenAI ChatKit

---

### 3. Non-Functional Requirements
- Response time <2 seconds
- API keys in `.env`
- Modular, type-annotated, clean code
- Scalable to multiple concurrent requests

---

### 4. Tech Stack
- Python 3.12, FastAPI, Uvicorn
- Cohere Embed API
- OpenAI Agents SDK / ChatKit SDKs
- Neon Serverless Postgres
- Qdrant Cloud Free Tier

---

### 5. Directory Structure
backend/
├── main.py
├── api/
│ ├── query.py
│ └── text_selection.py
├── embeddings/
│ └── cohere_embed.py
├── db/
│ └── neon_postgres.py
└── vectorstore/
└── qdrant_client.py

yaml
Copy code

---

### 6. Evaluation Criteria
- Accurate RAG responses
- Fast semantic search
- Clean, maintainable backend code
2. /sp.specify.frontend
Focus: Docusaurus integration + Chat widget"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Basic Query Functionality (Priority: P1)

A user wants to ask a question about the Physical AI & Humanoid Robotics textbook content. They submit their query to the RAG chatbot, and receive an accurate response based on the textbook content with proper citations.

**Why this priority**: This is the core functionality that enables users to get answers to their questions from the textbook content, providing immediate value.

**Independent Test**: Can be fully tested by sending a query to the backend API and verifying that it returns a relevant response based on the textbook content.

**Acceptance Scenarios**:

1. **Given** a user has a question about the textbook content, **When** they submit the query to the backend API, **Then** the system returns a relevant response with proper citations to the source material.
2. **Given** a user submits a query that cannot be answered with available content, **When** they send the request to the backend API, **Then** the system returns a helpful response indicating the information is not available in the current content.

---

### User Story 2 - Text Selection Query (Priority: P2)

A user has selected specific text from the textbook and wants to get more information about it. They submit the selected text to the RAG system, and receive contextually relevant information that expands on the selected content.

**Why this priority**: This enhances the user experience by allowing them to get more detailed information about specific parts of the textbook they're interested in.

**Independent Test**: Can be fully tested by sending selected text to the text selection endpoint and verifying that it returns relevant contextual information.

**Acceptance Scenarios**:

1. **Given** a user has selected text from the textbook, **When** they submit it to the text selection endpoint, **Then** the system returns relevant contextual information that expands on the selected content.

---

### User Story 3 - Content Embedding and Storage (Priority: P3)

The system needs to process the textbook content to generate embeddings using Cohere, store these embeddings in Qdrant for fast retrieval, and maintain metadata in Neon Postgres.

**Why this priority**: This is essential infrastructure that enables the query functionality but can be built independently and tested separately.

**Independent Test**: Can be fully tested by processing textbook content and verifying that embeddings are properly generated and stored in both Qdrant and Postgres.

**Acceptance Scenarios**:

1. **Given** new textbook content is available, **When** the embedding process runs, **Then** the system generates embeddings and stores them in Qdrant with corresponding metadata in Postgres.

---

### Edge Cases

- What happens when the query contains no meaningful content (empty or gibberish)?
- How does the system handle extremely long queries that exceed API limits?
- What happens when the Cohere API or Qdrant is temporarily unavailable?
- How does the system handle concurrent requests to ensure performance?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a `/query` endpoint that accepts user questions and returns relevant responses based on textbook content
- **FR-002**: System MUST provide a `/text-selection-query` endpoint that accepts selected text and returns contextual information
- **FR-003**: System MUST generate embeddings for textbook content using Cohere API
- **FR-004**: System MUST store embeddings in Qdrant vector database for semantic search
- **FR-005**: System MUST store content metadata in Neon Postgres database
- **FR-006**: System MUST perform RAG orchestration using OpenAI ChatKit to generate responses
- **FR-007**: System MUST return source citations with each response to maintain academic integrity
- **FR-008**: System MUST handle API authentication using keys stored in environment variables

### Key Entities

- **Query**: A user's question or request for information, containing text content and optional context
- **Textbook Content**: Structured educational material from the Physical AI & Humanoid Robotics textbook, including modules, chapters, and sections
- **Embedding**: Vector representation of text content generated by Cohere API for semantic similarity matching
- **Response**: Generated answer to user queries, containing relevant information and source citations

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive relevant responses to their queries within 2 seconds in 95% of cases
- **SC-002**: System can handle 100 concurrent requests without degradation in response time
- **SC-003**: 90% of generated responses contain accurate information that directly addresses the user's query
- **SC-004**: Response accuracy is validated against source textbook content with 95% precision
- **SC-005**: All responses include proper citations to specific textbook sections when relevant information exists
