# Tasks: Book Intelligence Agent

**Feature**: Book Intelligence Agent
**Spec**: [specs/002-book-intelligence-agent/spec.md](specs/002-book-intelligence-agent/spec.md)
**Plan**: [specs/002-book-intelligence-agent/plan.md](specs/002-book-intelligence-agent/plan.md)

## Dependencies
- Backend RAG Chatbot API (completed)
- Qdrant vector database with book content
- Neon Postgres for conversation storage

## Implementation Strategy
Implement upsert logic for conversation-related entities to prevent duplicate records. Each user story will be developed independently with comprehensive testing. The implementation will follow the MVP approach by first implementing core conversation functionality, then adding analytics and advanced features.

---

## Phase 1: Setup Tasks

- [X] T001 Set up project structure per implementation plan in specs/002-book-intelligence-agent/
- [X] T002 [P] Install required dependencies for conversation services in backend/requirements.txt
- [X] T003 [P] Create conversation models in backend/models/conversation.py
- [X] T004 [P] Create conversation service interface in backend/services/conversation_service.py
- [X] T005 [P] Update config to include conversation-related settings in backend/config.py

## Phase 2: Foundational Tasks

- [X] T006 Implement ConversationThread model with upsert logic in backend/models/conversation.py
- [X] T007 Implement ConversationMessage model with upsert logic in backend/models/conversation.py
- [X] T008 Implement UserAnalytics model with upsert logic in backend/models/conversation.py
- [X] T009 Create conversation database tables in backend/db/neon_postgres.py
- [X] T010 [P] Create conversation API endpoints in backend/api/conversation.py

## Phase 3: [US1] Conversation Thread Management

**User Story**: As a user, I want to create and manage conversation threads so that my interactions with the Book Intelligence Agent are preserved across sessions.

**Independent Test Criteria**: Verify that conversation threads can be created, retrieved, and updated without creating duplicate records.

- [X] T011 [US1] Implement ConversationThread upsert in NeonPostgresDB with ON CONFLICT handling in backend/db/neon_postgres.py
- [X] T012 [US1] Implement ConversationThread retrieval methods in backend/db/neon_postgres.py
- [X] T013 [US1] Create ConversationThread API endpoint with upsert logic in backend/api/conversation.py
- [X] T014 [US1] Test ConversationThread creation and update without duplicates
- [X] T015 [US1] Test ConversationThread retrieval by ID

## Phase 4: [US2] Conversation Message Management

**User Story**: As a user, I want to store and retrieve conversation messages so that I can see the history of my interactions with the Book Intelligence Agent.

**Independent Test Criteria**: Verify that conversation messages can be stored and retrieved with proper upsert logic to prevent duplicates.

- [X] T016 [US2] Implement ConversationMessage upsert in NeonPostgresDB with ON CONFLICT handling in backend/db/neon_postgres.py
- [X] T017 [US2] Implement ConversationMessage retrieval methods in backend/db/neon_postgres.py
- [X] T018 [US2] Create ConversationMessage API endpoint with upsert logic in backend/api/conversation.py
- [X] T019 [US2] Test ConversationMessage storage without creating duplicates
- [X] T020 [US2] Test ConversationMessage retrieval by conversation ID

## Phase 5: [US3] User Analytics Tracking

**User Story**: As a system administrator, I want to track user analytics with upsert logic so that duplicate analytics records are not created when users refresh or reconnect.

**Independent Test Criteria**: Verify that user analytics can be tracked and updated without creating duplicate records.

- [X] T021 [US3] Implement UserAnalytics upsert in NeonPostgresDB with ON CONFLICT handling in backend/db/neon_postgres.py
- [X] T022 [US3] Implement UserAnalytics retrieval methods in backend/db/neon_postgres.py
- [X] T023 [US3] Create UserAnalytics API endpoint for tracking interactions in backend/api/analytics.py
- [X] T024 [US3] Test UserAnalytics tracking without creating duplicates
- [X] T025 [US3] Test UserAnalytics retrieval and aggregation

## Phase 6: [US4] Conversation Integration with Book Intelligence Agent

**User Story**: As a user, I want my conversations to be integrated with the Book Intelligence Agent so that context is preserved and analytics are tracked.

**Independent Test Criteria**: Verify that the Book Intelligence Agent properly creates conversation records with upsert logic and tracks analytics.

- [X] T026 [US4] Update RAG service to create conversation records with upsert logic in backend/services/rag_service.py
- [X] T027 [US4] Update query API to integrate conversation tracking in backend/api/query.py
- [X] T028 [US4] Update text selection API to integrate conversation tracking in backend/api/text_selection.py
- [X] T029 [US4] Test complete conversation flow with upsert logic
- [X] T030 [US4] Test analytics tracking during conversation flow

## Phase 7: [US5] Conversation History and Context Management

**User Story**: As a user, I want the Book Intelligence Agent to remember previous parts of the conversation even if I refresh the page, so that I can have a continuous conversation experience.

**Independent Test Criteria**: Verify that conversation history is properly retrieved and used as context for new queries.

- [X] T031 [US5] Implement conversation history retrieval with upsert logic in backend/services/conversation_service.py
- [X] T032 [US5] Update context assembly to include conversation history in backend/services/rag_service.py
- [X] T033 [US5] Test conversation continuity across page refreshes
- [X] T034 [US5] Test context preservation for follow-up questions
- [X] T035 [US5] Test conversation history performance with large conversations

## Phase 8: Polish & Cross-Cutting Concerns

- [X] T036 Add comprehensive logging for upsert operations in all services
- [X] T037 Add error handling and validation for all upsert operations
- [X] T038 Update documentation for conversation APIs in docs/
- [X] T039 Create integration tests for complete conversation flow
- [X] T040 Performance test upsert operations under load

---

## Dependencies

**User Story Completion Order**:
- US1 (Conversation Thread Management) must be completed before US2, US4, US5
- US2 (Conversation Message Management) must be completed before US4, US5
- US3 (User Analytics Tracking) can be completed in parallel with other stories
- US4 (Conversation Integration) depends on US1 and US2
- US5 (Conversation History) depends on US1, US2, and US4

## Parallel Execution Examples

**Per User Story**:
- US1: T011-T012 can run in parallel with T013-T014
- US2: T016-T017 can run in parallel with T018-T019
- US3: T021-T022 can run in parallel with T023-T024
- US4: T026 can run in parallel with T027-T028
- US5: T031 can run in parallel with T032

## MVP Scope

The MVP will include US1 (Conversation Thread Management) and US2 (Conversation Message Management) to provide basic conversation persistence functionality. This will allow users to have their conversations preserved across sessions without duplicates.
