# Implementation Plan: Backend Todo API

## Technical Context

**Project**: Backend Implementation for Phase II Todo Web Application
**Phase**: Backend-only development (frontend already completed)
**Current Date Reference**: January 12, 2026
**Development Paradigm**: Agentic + Spec-Driven (Claude Code + Spec-Kit Plus)
**Goal**: Implement secure, scalable backend API for todo application

### Architecture Overview

**High-Level Architecture Sketch**
```
Full-Stack-Web-Application/                        ← monorepo ROOT (where ALL credentials live)
├── .env                               ← ★ Main secrets file (all services read from here)
├── .env.example                       ← Template for contributors (no real values)
├── .gitignore                         ← Ignores .env, .env.local, etc.
├── docker-compose.yml                 ← Uses root .env via env_file
├── .spec-kit/
│   └── config.yaml
├── specs/
├── frontend/                          ← already built
├── backend/
│   ├── main.py
│   ├── models.py
│   ├── schemas.py
│   ├── crud/
│   │   └── tasks.py
│   ├── routes/
│   │   └── tasks.py
│   ├── db.py
│   ├── auth.py                        ← reads BETTER_AUTH_SECRET from os.environ
│   ├── utils.py
│   ├── requirements.txt
│   ├── CLAUDE.md
│   └── alembic/                       ← migrations (optional future)
└── README.md
```

### Data Flow & Integration Points

**User → JWT (from Better Auth) → API (verify token) → DB (filter by user_id)**
→ Server responses with Pydantic models
→ Optimistic UI updates (future frontend integration)

### Section Structure (Folder & Feature Organization)

```
backend/
├── main.py             # FastAPI app, middleware, lifespan
├── models.py           # SQLModel table definitions (User, Task)
├── schemas.py          # Pydantic models for requests/responses
├── crud/               # Database operations
│   └── tasks.py        # Task-specific CRUD functions
├── routes/             # API endpoints
│   └── tasks.py        # Task endpoints with authentication
├── db.py               # Database engine/session setup
├── auth.py             # JWT middleware and user extraction
├── utils.py            # Helper functions (recurrence logic, etc.)
├── requirements.txt    # Python dependencies
└── CLAUDE.md           # Backend development guidelines
```

### Architectural & Design Decisions

| Decision | Options Considered | Chosen | Tradeoffs / Rationale |
|----------|-------------------|---------|----------------------|
| Authentication method | Session-based / JWT with PyJWT / OAuth2 | JWT with PyJWT | Stateless, works with Better Auth, minimal dependencies |
| Database session management | Global session / per-request dependency | Per-request via FastAPI Depends | Thread-safe, proper async support, transaction scope |
| ORM/ODM choice | SQLAlchemy / SQLModel / TortoiseORM | SQLModel | Combines SQLAlchemy + Pydantic, type safety, async support |
| Dependency injection pattern | Global vars / middleware / FastAPI Depends | FastAPI Depends with yield | Type-safe, proper cleanup, async-friendly |
| Error handling approach | Exceptions / Result type / Custom handlers | HTTPException + custom handlers | FastAPI-native, clear error responses |
| Input validation | Marshmallow / Pydantic / Manual checks | Pydantic schemas | Type safety, FastAPI integration, automatic docs |
| Rate limiting approach | In-memory / Redis / Database | In-memory with Starlette | Simple for initial implementation, can upgrade later |

### Agent Structure & Responsibilities

| Agent Name | Primary Responsibility | Preferred Spec References | Output Style |
|------------|------------------------|---------------------------|--------------|
| claude/database-architect | Models, relationships, indexes, migration strategy | @specs/database/schema.md | SQLModel-first approach |
| claude/api-designer | Endpoint logic, query building, response shaping | @specs/api/rest-endpoints.md | OpenAPI-friendly patterns |
| claude/authentication-specialist | JWT middleware, user dependency, ownership checks | @specs/features/authentication.md | Security-first approach |
| claude/query-optimizer | Dynamic filters, sorting, pagination, indexing advice | Task filtering & sorting requirements | SQL performance focused |
| claude/recurrence-engineer | Recurring task logic, next occurrence calculation | Advanced features spec | Algorithm + edge-case list |
| claude/feature-breaker | Breaks implementation into smallest safe tasks | All backend specs | Numbered, prioritized tasks |

## Constitution Check

Based on the project constitution, this plan must:
- Follow Spec-Driven Development principles (spec already created)
- Maintain modularity and reusability in component design
- Implement security-first approach with JWT authentication
- Prioritize user-centric design with proper error handling
- Use efficient Agentic Dev Stack workflow (Spec → Plan → Tasks → Implement)
- Maintain visual consistency in API design and documentation

## Phase Gates

### Gate 1: Architecture Review
- [x] Architecture follows FastAPI best practices
- [x] Component structure supports reusability and maintainability
- [x] Security measures implemented for authentication and data handling
- [x] Tech stack decisions justified with tradeoffs documented
- [x] Database schema aligns with functional requirements

### Gate 2: Design Review
- [x] Data model aligns with functional requirements
- [x] API contracts defined for all required operations
- [x] Error handling strategies documented
- [x] Performance considerations addressed

## Phase 0: Research & Discovery

### Research Tasks

1. **Authentication Implementation Research**
   - Research JWT implementation with PyJWT in FastAPI
   - Document how to verify Better Auth JWT tokens
   - Identify best practices for token validation and user extraction

2. **Database Connection Research**
   - Research async PostgreSQL connection with Neon using asyncpg
   - Document best practices for connection pooling
   - Identify optimal configuration for serverless database

3. **SQLModel Implementation Research**
   - Research SQLModel best practices for table definitions
   - Document relationship patterns and indexing strategies
   - Identify optimal query patterns for filtering and sorting

4. **Rate Limiting Implementation Research**
   - Research rate limiting options for FastAPI applications
   - Document in-memory vs Redis approaches
   - Identify suitable packages for rate limiting

5. **Recurrence Logic Research**
   - Research best practices for recurring task implementation
   - Document algorithms for calculating next occurrence dates
   - Identify patterns for handling recurrence edge cases

## Phase 1: Design & Architecture

### 1.1 Data Model Design

Based on the feature specification, the following entities will be implemented:

#### User Entity (referenced from Better Auth)
- id: str (unique identifier from Better Auth)
- email: str (unique, from Better Auth)
- name: str (from Better Auth)
- created_at: datetime (from Better Auth)

#### Task Entity
- id: int (primary key, auto-increment)
- user_id: str (foreign key to User.id, required)
- title: str (required, 1-100 characters)
- description: str (optional, max 5000 characters)
- completed: bool (default False)
- priority: str (enum: 'high', 'medium', 'low', optional)
- tags: List[str] (array of strings, optional)
- due_date: datetime (optional, stored in UTC)
- recurrence: str (enum: 'none', 'daily', 'weekly', default 'none')
- created_at: datetime (auto-set)
- updated_at: datetime (auto-update)

### 1.2 API Contract Design

The backend will expose the following endpoints:

#### Base URL: /api/{user_id}/

All endpoints require JWT authentication and enforce user_id matching.

#### Task Management Endpoints
- `GET /tasks` - Get all user tasks with query params for filtering/sorting
- `POST /tasks` - Create new task for user
- `GET /tasks/{id}` - Get single task details
- `PUT /tasks/{id}` - Update task
- `DELETE /tasks/{id}` - Delete task
- `PATCH /tasks/{id}/complete` - Toggle task completion

#### Query Parameters for GET /tasks
- status: 'all'/'pending'/'completed' (default: 'all')
- search: keyword to search in title/description
- priority: 'high'/'medium'/'low' (optional filter)
- date_from: ISO datetime for date range filtering
- date_to: ISO datetime for date range filtering
- sort: 'due_date'/'priority'/'title' (default: 'created_at')
- order: 'asc'/'desc' (default: 'desc')

### 1.3 Component Architecture

#### Models Layer (models.py)
- User model (reference to Better Auth user structure)
- Task model (with all required fields and relationships)
- Database engine and session setup

#### Schemas Layer (schemas.py)
- TaskCreate schema (for POST requests)
- TaskUpdate schema (for PUT requests, partial updates)
- TaskResponse schema (for API responses)
- TaskToggleComplete schema (for PATCH requests)

#### Database Layer (crud/tasks.py)
- get_tasks() - Retrieve tasks with filters, search, and sorting
- create_task() - Create new task with user association
- get_task() - Get single task by ID with user verification
- update_task() - Update task fields
- delete_task() - Delete task
- toggle_task_completion() - Toggle completion status and handle recurrence

#### Routes Layer (routes/tasks.py)
- Authentication middleware enforcement
- Endpoint definitions matching API contracts
- Request/response validation with Pydantic schemas
- Error handling and proper HTTP status codes

#### Authentication Layer (auth.py)
- JWT token verification middleware
- Current user dependency for endpoint protection
- User ID extraction and validation

#### Utilities Layer (utils.py)
- Recurrence calculation functions
- Date/time utility functions
- Rate limiting implementation

## Phase 2: Implementation Strategy

### 2.1 Development Sequencing

#### Setup Phase
1. Create backend directory structure
2. Set up requirements.txt with dependencies
3. Create root .env and .env.example files
4. Configure database connection in db.py

#### Authentication Implementation
1. Implement JWT verification in auth.py
2. Create current_user dependency
3. Set up middleware for token validation

#### Core Task Functionality
1. Define Task model in models.py
2. Create Pydantic schemas in schemas.py
3. Implement basic CRUD operations in crud/tasks.py
4. Create task endpoints in routes/tasks.py
5. Connect routes to main application

#### Intermediate Features
1. Implement filtering and search functionality
2. Add sorting capabilities
3. Implement priority and tags handling
4. Add date range filtering

#### Advanced Features
1. Implement recurrence logic
2. Add rate limiting
3. Enhance error handling
4. Add comprehensive validation

#### Quality Assurance
1. Add unit tests for all components
2. Implement integration tests
3. Performance testing and optimization
4. Security testing

### 2.2 Quality Validation Strategy

#### Type Safety
- mypy --strict must pass (CI check)
- Pydantic schemas for every request/response
- SQLModel type annotations for database models

#### Unit / Integration Tests (pytest)
- Core CRUD operations (create, read, update, delete)
- Authentication and authorization checks
- Filtering, search, and sorting functionality
- Recurrence logic validation

#### Acceptance Criteria Checklist (manual + automated)
- Auth: All endpoints require JWT, user_id in token matches URL param, 401/403 for invalid requests
- Create: Required fields enforced, new task appears with correct user association
- Read: Users can only access their own tasks, filtering works correctly
- Update: Partial updates allowed, user isolation maintained
- Delete: Tasks are properly removed (hard delete), user isolation maintained
- Complete toggle: Status updates correctly, recurrence logic works when applicable
- Search & Filter: Keyword search works across fields, filters apply correctly
- Sort: Tasks sort by specified criteria correctly
- Recurring: New tasks created correctly when recurring tasks completed

#### Performance Checklist
- API responses under 200ms for typical queries
- Database queries use proper indexing
- Efficient filtering and sorting algorithms
- Proper async handling for concurrent requests

#### Security Checklist
- All endpoints require authentication
- User data isolation enforced (user A cannot access user B's data)
- Input validation prevents injection attacks
- Rate limiting prevents abuse
- JWT tokens properly verified

## Implementation Artifacts

### Generated Files
- `data-model.md` - Detailed data model specification
- `contracts/` - API contract definitions
- `quickstart.md` - Getting started guide for developers
- `backend/CLAUDE.md` - Agent instructions for backend development