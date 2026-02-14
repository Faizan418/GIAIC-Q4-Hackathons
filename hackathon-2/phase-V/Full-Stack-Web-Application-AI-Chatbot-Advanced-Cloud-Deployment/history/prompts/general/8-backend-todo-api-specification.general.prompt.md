---
id: 8
title: Backend Todo API Specification
stage: general
date: 2026-01-12
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: unknown
command: sp.specify
labels: ["specification", "backend", "todo-api", "fastapi"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - E:\Phase 2\Full-Stack-Web-Application\specs\1-backend-todo-api\spec.md
 - E:\Phase 2\Full-Stack-Web-Application\specs\1-backend-todo-api\checklists\requirements.md
tests:
 - none
---

## Prompt

Backend Specification for Phase II: Todo Full-Stack Web Application
Project Overview
This specification defines the backend implementation for the Todo App in Phase II, building a secure, RESTful API to support the completed frontend. Focus is exclusively on the backend using Python FastAPI, SQLModel for ORM, and Neon Serverless PostgreSQL for storage. The backend will handle multi-user data isolation via JWT authentication from Better Auth, integrating with frontend API calls. All features—basic, intermediate, and advanced—are required, ensuring persistent storage, validation, and efficient queries. Development follows spec-driven approach: Write spec → Generate plan → Break into tasks → Implement via Claude Code. No manual coding.
Target Audience: Developers implementing via Claude Code; ensures API aligns with frontend needs.
Focus: Provide robust CRUD operations, user authentication verification, and advanced logic like recurring tasks and reminders (server-side where applicable). Enforce security, data validation, and scalability.
User Stories (Backend Perspective)

As the backend, I verify JWT on all requests and filter data by authenticated user.
As the backend, I handle task creation, retrieval, updates, deletions, and status toggles with validation.
As the backend, I support priorities, tags, due dates, search/filter/sort queries.
As the backend, I manage recurring tasks (auto-rescheduling logic) and store reminders for frontend triggering.

Tech Stack (Backend Only)

Framework: FastAPI for async API routing and validation.
ORM: SQLModel for database models and queries (combines SQLAlchemy + Pydantic).
Database: Neon Serverless PostgreSQL for managed, scalable storage.
Authentication: JWT verification middleware (using PyJWT or similar) with shared BETTER_AUTH_SECRET.
Other: Pydantic for request/response models, SQLAlchemy for sessions, background tasks for any async operations (e.g., recurring auto-creation if scheduled).
Environment: Python 3.10+, dependencies in requirements.txt (fastapi, sqlmodel, uvicorn, psycopg2, pyjwt).

Folder Structure (Backend)
text/backend
├── main.py               # FastAPI app entry, middleware setup
├── models.py             # SQLModel table definitions (User, Task)
├── schemas.py            # Pydantic models for requests/responses
├── crud/                 # CRUD operations as functions
│   └── tasks.py          # Task-specific CRUD logic
├── routes/               # API routers
│   └── tasks.py          # Task endpoints
├── db.py                 # Database engine/session setup
├── auth.py               # JWT middleware and user extraction
├── utils.py              # Helpers (e.g., recurrence logic)
├── requirements.txt      # Dependencies
├── CLAUDE.md             # Backend guidelines
└── .env                  # Secrets (DATABASE_URL, BETTER_AUTH_SECRET)
Authentication Spec

Middleware: Extract JWT from Authorization header, verify signature with BETTER_AUTH_SECRET.
User Extraction: Decode token to get user_id, email; match against URL {user_id}.
Protection: Apply to all /api/{user_id}/* routes; return 401 on invalid/missing token.
User Model: Sync with Better Auth's users table (id, email, etc.); assume Better Auth handles user creation.
Isolation: All queries filter by user_id; prevent cross-user access.

Database Schema

Tables:
users (from Better Auth): id (str, PK), email (str, unique), name (str), created_at (timestamp).
tasks: id (int, PK), user_id (str, FK to users.id), title (str, not null), description (text, nullable), completed (bool, default False), priority (enum: 'high', 'medium', 'low', nullable), tags (array[str], nullable), due_date (datetime, nullable), recurrence (enum: 'none', 'daily', 'weekly', nullable), created_at (timestamp), updated_at (timestamp).

Indexes: user_id (for fast filtering), completed (status queries), due_date (sorting/filtering), priority.
Setup: Use SQLModel metadata.create_all() on startup; DATABASE_URL from env.

API Endpoints Spec

Base: /api/{user_id}/
All require JWT; filter by user_id.
Responses: JSON with Pydantic models; errors via HTTPException (400 for validation, 404 for not found, etc.).
GET /tasks: List tasks. Query params: status ('all'|'pending'|'completed'), search (keyword), priority ('high'|'medium'|'low'), date_from/date_to (ISO datetime), sort ('due_date'|'priority'|'title' asc/desc). Return list of Task schemas.
POST /tasks: Create task. Body: title (required), description, priority, tags (list[str]), due_date, recurrence. Associate with user_id. Return created Task.
GET /tasks/{id}: Get single task details. Return Task or 404.
PUT /tasks/{id}: Update task. Body: partial updates (title, etc.). Return updated Task.
DELETE /tasks/{id}: Delete task. Return 204 No Content.
PATCH /tasks/{id}/complete: Toggle completed. Optional body: {completed: bool}. For recurring: If completed and recurring, create new task with next due_date (e.g., +1 day/week). Return updated Task.

CRUD Operations (in crud/tasks.py)

Create: Validate body with Pydantic, insert to DB with user_id.
Read (List): Build dynamic SQL query with filters (WHERE user_id = ?, AND completed = ? if status, ILIKE for search on title/description, BETWEEN for dates, ORDER BY for sort). Use SQLModel select() with where clauses.
Read (Single): SELECT WHERE id = ? AND user_id = ?.
Update: Partial update with setattr; update updated_at.
Delete: DELETE WHERE id = ? AND user_id = ?.
Toggle Complete: UPDATE completed = NOT completed; if recurring, background task to insert new task with adjusted due_date (calculate next: due_date + interval).

Feature Implementation (Basic Level)

Add/Delete/Update/View/Mark Complete: As per endpoints. Validation: Title 1-200 chars, description <=1000. Use Pydantic validators.

Feature Implementation (Intermediate Level)

Priorities & Tags: Store in DB; filter/sort support in GET (e.g., WHERE priority = ?, ARRAY_OVERLAPS for tags).
Search & Filter: ILIKE for keyword (title || description), WHERE clauses for status/priority/dates.
Sort: Dynamic ORDER BY based on param (e.g., CASE for priority ordinal: high=3, medium=2, low=1).

Feature Implementation (Advanced Level)

Due Dates & Reminders: Store due_date; no server-side notifications (frontend handles browser alerts). Support date filtering/sorting.
Recurring Tasks: On toggle complete (if recurrence != 'none'), compute next_due = due_date + delta (daily: +1 day, weekly: +7 days). Insert clone task with new due_date, completed=False. Use FastAPI BackgroundTasks for async insert.

Utilities and Middleware

auth.py: Dependency for current_user: Decode JWT, raise 401/403 if invalid or user_id mismatch.
db.py: Async engine with create_engine(DATABASE_URL, echo=True for dev).
utils.py: Recurrence delta calculator (timedelta functions).
Sessions: Use sessionmaker for scoped sessions in dependencies.

Acceptance Criteria

All endpoints tested with valid/invalid JWT (401/403).
Data isolated: User A can't access User B's tasks.
Validation: Reject invalid bodies (400 with details).
Queries efficient: Use indexes; limit results (pagination if >100, but optional).
Recurring: Correctly clones on complete; no infinite loops.
Errors: Graceful handling (e.g., DB errors → 500).

Success Criteria

API fully supports frontend features.
Secure: JWT enforced, no vulnerabilities.
Performant: <200ms responses for typical queries.
Scalable: Neon handles growth; async where needed.

Constraints

Word count: This spec under 1000 (actual: 912).
No frontend specs or implementation.
Use only specified stack; no extra deps without necessity.
Timeline: Implement after spec approval, via Claude Code.
Testing: Pytest for units (CRUD funcs), integration (endpoints).

Not Building

Frontend UI or API client.
Custom user management (rely on Better Auth).
Real-time features (e.g., websockets).
Deployment (handle later).
Advanced scheduling (e.g., cron for reminders).

## Response snapshot

Created comprehensive backend specification for the Todo API with user stories, functional requirements, success criteria, and entity definitions.

## Outcome

- ✅ Impact: Established clear requirements for backend API implementation with proper authentication, data isolation, and feature completeness
- 🧪 Tests: none
- 📁 Files: Created spec.md and requirements checklist for backend todo API
- 🔁 Next prompts: /sp.plan to generate implementation plan
- 🧠 Reflection: Specification provides complete roadmap for backend development with JWT authentication and user data isolation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate implementation plan based on this specification