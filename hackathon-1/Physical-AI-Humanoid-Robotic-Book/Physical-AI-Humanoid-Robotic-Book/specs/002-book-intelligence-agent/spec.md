# Book Intelligence Agent

## Feature Description

Create a "Book Intelligence Agent" that serves as an expert assistant dedicated to answering questions exclusively based on the content of a published book stored in a knowledge base and any specific text snippets provided by the user.

## User Scenarios & Testing

### Primary User Flow
1. User asks a question about the book content
2. System searches the Qdrant vector database for relevant passages
3. If user provides a "Selected Snippet," prioritize this text above all other information
4. System responds with answer grounded only in the retrieved context or selected snippet
5. If information is not available in the book, system politely states this limitation

### Edge Cases
- Question cannot be answered with available book content
- User provides a very short selected snippet that doesn't provide full context
- Ambiguous questions that could have multiple interpretations in the book

## Functional Requirements

1. **Knowledge Retrieval**: The system must search the Qdrant vector database to retrieve the most relevant passages when a user asks a question.

2. **Context Prioritization**: When a user provides a specific "Selected Snippet," the system must prioritize this text above all other information during response generation.

3. **Grounding Enforcement**: The system must answer ONLY using the retrieved context or the selected snippet. If the answer is not contained within the provided materials, the system must politely state that the information is not available in the book.

4. **Accuracy Assurance**: The system must not hallucinate or bring in outside knowledge (e.g., "According to the internet...").

5. **Citation Capability**: When possible, the system must refer to the chapter or section name if available in the metadata.

6. **Tone Consistency**: The system must maintain a professional, academic, yet accessible tone that reflects the author's voice.

7. **Reasoning Validation**: Before answering, the system must verify if the retrieved context directly addresses all parts of the user's query.

8. **Structured Response**: The system must use Markdown for structure (headings, bullet points) when appropriate.

9. **Context Request**: If a user's selected text is too short to provide a full answer, the system must ask for more context or offer to search the full book database.

## Success Criteria

- Users receive accurate answers based solely on book content with 95% accuracy
- System correctly identifies when information is not available in the book 99% of the time
- User satisfaction rating of 4.0/5.0 or higher for response accuracy and relevance
- Response time under 3 seconds for 90% of queries
- System maintains professional and consistent tone in 100% of responses
- Users can successfully get answers to their book-related questions without receiving hallucinated information

## Key Entities

- **Book Intelligence Agent**: The AI system that processes user queries
- **Qdrant Vector Database**: The storage system for book content embeddings
- **User Queries**: Questions posed by users about book content
- **Selected Snippets**: Specific text provided by users for prioritized context
- **Retrieved Context**: Relevant passages retrieved from the book database
- **Metadata**: Chapter/section information associated with book content

## Assumptions

- The Qdrant vector database is already populated with book content embeddings
- The `search_book_content` tool is available and functional
- Book content is properly indexed with metadata including chapter/section names
- Users have a basic understanding of how to interact with AI assistants
- The system has access to appropriate error handling mechanisms

## Dependencies

- Qdrant vector database with book content
- `search_book_content` tool functionality
- Book content properly indexed and stored
- Vector embedding models for content retrieval
