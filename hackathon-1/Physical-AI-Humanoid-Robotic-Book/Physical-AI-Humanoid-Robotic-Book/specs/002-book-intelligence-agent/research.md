# Research: Book Intelligence Agent

## Decision: Global vs. Targeted Search Implementation
**Rationale**: The system needs to handle two distinct search scenarios: 1) Global search for general questions using Qdrant vector database, and 2) Targeted search prioritizing user-selected text snippets. This requires a dual-approach where the system can conditionally switch between full database search and snippet-prioritized responses.
**Alternatives considered**:
- Single unified search approach that treats all inputs the same
- Separate API endpoints for each search type
- Stateful session that remembers user preferences
**Chosen approach**: Conditional logic in the query handler that checks for provided snippets and prioritizes them accordingly.

## Decision: Context Prioritization Strategy
**Rationale**: When users provide selected snippets, these must be prioritized above all other information to meet functional requirement #2. This requires a clear hierarchy in the context assembly process.
**Alternatives considered**:
- Weighted scoring system that combines snippets with database results
- Hard prioritization where snippets completely override database results
- Hybrid approach with confidence scoring
**Chosen approach**: Hard prioritization where if a selected snippet is provided, it becomes the primary context and database search is either skipped or used as secondary context.

## Decision: Grounding Enforcement Mechanism
**Rationale**: To prevent hallucination and ensure responses are grounded only in provided materials (requirement #3), the system needs a verification step before generating responses.
**Alternatives considered**:
- LLM self-verification prompts that ask the model to check its response
- Post-generation validation that checks response content against provided context
- Strict context window management that limits the LLM's input
**Chosen approach**: Combination of strict context window management and LLM self-verification prompts with grounding enforcement instructions.

## Decision: Conversation Thread Storage
**Rationale**: Using Neon Postgres to store conversation threads allows the agent to remember previous parts of the conversation even if the user refreshes the page (as suggested in implementation tip).
**Alternatives considered**:
- Client-side storage (localStorage/sessionStorage)
- Redis cache for temporary conversation storage
- In-memory storage with export capability
**Chosen approach**: Neon Postgres with conversation entities that track user sessions and query history.

## Decision: Citation Capability Implementation
**Rationale**: To meet requirement #5, the system must refer to chapter or section names when available in metadata. This requires metadata to be stored alongside the book content embeddings.
**Alternatives considered**:
- Simple proximity-based citations based on embedding similarity
- Metadata-rich embeddings with chapter/section information
- Post-processing of responses to add citation information
**Chosen approach**: Metadata-rich embeddings that include chapter/section information in the Qdrant database.

## Decision: Reasoning Validation Process
**Rationale**: To meet requirement #7, the system must verify if retrieved context directly addresses all parts of the user's query before responding.
**Alternatives considered**:
- Simple keyword matching between query and context
- Semantic similarity scoring
- Multi-step validation with LLM reasoning
**Chosen approach**: Multi-step validation with LLM reasoning that checks if the context sufficiently addresses the query before generating a response.

## Decision: Error Handling for Insufficient Information
**Rationale**: When information is not available in the book (requirement #5), the system must politely state this limitation.
**Alternatives considered**:
- Generic "information not found" responses
- Detailed explanation of why the information wasn't found
- Suggestions for related topics that might be helpful
**Chosen approach**: Polite acknowledgment with explanation that the information is not available in the book, with optional suggestions for rephrasing the question.

## Decision: Selected Text Short Context Handling
**Rationale**: To meet requirement #9, when user's selected text is too short to provide a full answer, the system should ask for more context or offer to search the full book database.
**Alternatives considered**:
- Automatically expanding the selected text to include surrounding context
- Requesting additional user input without offering alternatives
- Providing best-effort answer with disclaimer
**Chosen approach**: Explicit request for more context with option to search the full database.
