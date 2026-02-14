# Implementation Tasks: Full-Stack Todo App Integration

**Feature**: Full-Stack Todo App Integration
**Spec File**: [spec.md](./spec.md)
**Plan File**: [plan.md](./plan.md)
**Created**: 2026-01-12
**Status**: Implementation Complete
**Author**: Claude

## Implementation Strategy

This task breakdown implements the full-stack integration of the todo application, connecting the frontend and backend systems to enable real data flow instead of mock data. The implementation follows an incremental approach, starting with foundational setup and progressing through user stories in priority order.

**MVP Scope**: Complete User Story 1 (Connect Frontend to Backend API) to establish the core data flow.

**Architecture**: Monorepo with separate frontend (Next.js) and backend (FastAPI) directories
**Backend**: Python FastAPI with SQLModel, PostgreSQL database
**Frontend**: Next.js 16+ with Better Auth for authentication
**Dependency Management**: uv for backend dependencies
**Authentication**: JWT tokens from Better Auth for user sessions

## Dependencies

- User Story 4 (Development Environment Setup) must be completed before other stories
- User Story 2 (Authentication Flow) is required for User Stories 1 and 3
- User Story 1 (Frontend-Backend Connection) is required for User Story 3 (Full Feature Integration)

## Parallel Execution Opportunities

Each user story can largely be worked on in parallel after foundational setup:
- Backend API development (routes, models, auth)
- Frontend API client implementation (api.ts)
- Frontend component updates (removing mock data)
- Better Auth configuration

## Phase 1: Setup (Project Initialization)

**Goal**: Establish the project structure and foundational configuration

- [X] T001 Create root pyproject.toml with uv workspace configuration
- [X] T002 Create backend/pyproject.toml with required dependencies
- [X] T003 Set up root .env file with secrets placeholder
- [X] T004 Create .env.example with template values
- [X] T005 Initialize uv workspace and sync dependencies
- [X] T006 Create missing backend directories (alembic/, tests/)
- [X] T007 Create backend test files (tests/__init__.py, tests/test_tasks.py)

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Implement core infrastructure required by all user stories

- [X] T010 [P] Update backend auth.py with JWT verification function
- [X] T011 [P] Update backend db.py to load environment variables from root .env
- [X] T012 [P] Update backend main.py to include router with proper prefix
- [X] T013 [P] Update backend routes to validate user_id from token against URL path
- [X] T014 [P] Update Better Auth configuration to use JWT plugin with shared secret
- [X] T015 [P] Create docker-compose.yml for local development setup

## Phase 3: [US1] Connect Frontend to Backend API (Priority: P1)

**Goal**: Enable real data flow between frontend and backend, replacing all mock data

**Independent Test**: Can be fully tested by logging into the application, creating a task, refreshing the page, and verifying the task still exists. This delivers the fundamental value of persistent task management.

- [X] T020 [P] [US1] Update frontend lib/api.ts with real backend API calls
- [X] T021 [US1] Remove all mock data from frontend components
- [X] T022 [US1] Update frontend task components to use real API data
- [X] T023 [US1] Update frontend task creation to use real API endpoint
- [X] T024 [US1] Update frontend task update/delete to use real API endpoints
- [X] T025 [US1] Implement proper error handling for API failures in frontend
- [X] T026 [US1] Test data persistence by creating task and refreshing page

## Phase 4: [US2] End-to-End Authentication Flow (Priority: P1)

**Goal**: Implement complete authentication flow with JWT token handling between frontend and backend

**Independent Test**: Can be fully tested by logging in, then making API calls to the backend and verifying that the JWT token is properly validated and the user can only access their own data.

- [X] T030 [P] [US2] Verify JWT token flow from Better Auth to backend API calls
- [X] T031 [US2] Implement user_id validation in backend routes (token vs URL)
- [X] T032 [US2] Test authentication with valid JWT token
- [X] T033 [US2] Test authentication rejection with invalid/expired token
- [X] T034 [US2] Verify user data isolation (users can only access own tasks)
- [X] T035 [US2] Implement proper error responses for authentication failures

## Phase 5: [US3] Full Feature Integration (Priority: P2)

**Goal**: Ensure all todo features work end-to-end with real data: create, complete, delete, update, priority, tags, due date, recurrence, search, filter, sort

**Independent Test**: Can be fully tested by using each feature individually with real data and verifying proper persistence and retrieval from the database.

- [X] T040 [P] [US3] Implement priority feature with real data flow
- [X] T041 [P] [US3] Implement tags feature with real data flow
- [X] T042 [P] [US3] Implement due date feature with real data flow
- [X] T043 [P] [US3] Implement recurrence feature with real data flow
- [X] T044 [P] [US3] Implement search feature with real data flow
- [X] T045 [P] [US3] Implement filter feature with real data flow
- [X] T046 [P] [US3] Implement sort feature with real data flow
- [X] T047 [US3] Test recurring task creation when completed
- [X] T048 [US3] Test advanced filtering and sorting with real data

## Phase 6: [US4] Development Environment Setup (Priority: P1)

**Goal**: Complete backend dependency management with uv and ensure consistent development environment

**Independent Test**: Can be fully tested by running the application with uv and verifying all dependencies are properly managed and the application starts successfully.

- [X] T050 [P] [US4] Complete uv workspace setup and dependency management
- [X] T051 [US4] Verify backend runs successfully with uv run commands
- [X] T052 [US4] Test docker-compose setup (if implemented)
- [X] T053 [US4] Run type checking (tsc --noEmit) on frontend
- [X] T054 [US4] Run static analysis (mypy) on backend
- [X] T055 [US4] Verify zero console errors during normal operation

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Address edge cases, performance, and final integration issues

- [X] T060 Add proper error handling for database unavailability
- [X] T061 Implement token refresh mechanism for long-running operations
- [X] T062 Configure CORS settings for frontend-backend communication
- [X] T063 Optimize database queries with proper indexing
- [X] T064 Add comprehensive logging for debugging
- [X] T065 Perform final end-to-end testing
- [X] T066 Clean up any remaining mock data or hardcoded values
- [X] T067 Verify all acceptance criteria are met
- [X] T068 Update README.md with uv commands and setup instructions

## Acceptance Criteria Verification

- [X] Real Neon DB has tasks after creation
- [X] No mock data in any file (verify with grep -r "mock" returns no results)
- [X] Frontend shows real tasks from PostgreSQL database
- [X] JWT auth works end-to-end (login → protected routes → API calls succeed)
- [X] All features work: create, complete, delete, update, priority, tags, due date, recurrence, search, filter, sort
- [X] Zero console errors appear in either frontend or backend
- [X] Backend runs successfully with uv dependency management
- [X] Application starts without errors via docker-compose or separate commands
- [X] Recurring tasks properly create new instances when completed
- [X] User data isolation is maintained (users only access own tasks)