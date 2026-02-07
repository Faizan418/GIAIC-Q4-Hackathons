# Backend Specification for Phase II: Todo Full-Stack Web Application

## Project Overview
This specification defines the backend implementation for the Todo App in Phase II, building a secure, RESTful API to support the completed frontend. Focus is exclusively on the backend using Python FastAPI, SQLModel for ORM, and Neon Serverless PostgreSQL for storage. The backend will handle multi-user data isolation via JWT authentication from Better Auth, integrating with frontend API calls. All features—basic, intermediate, and advanced—are required, ensuring persistent storage, validation, and efficient queries. Development follows spec-driven approach: Write spec → Generate plan → Break into tasks → Implement via Claude Code. No manual coding.

Target Audience: Developers implementing via Claude Code; ensures API aligns with frontend needs.

Focus: Provide robust CRUD operations, user authentication verification, and advanced logic like recurring tasks and reminders (server-side where applicable). Enforce security, data validation, and scalability.

## User Stories (Backend Perspective)

As the backend, I verify JWT on all requests and filter data by authenticated user.
As the backend, I handle task creation, retrieval, updates, deletions, and status toggles with validation.
As the backend, I support priorities, tags, due dates, search/filter/sort queries.
As the backend, I manage recurring tasks (auto-rescheduling logic) and store reminders for frontend triggering.

## User Scenarios & Testing

### Scenario 1: Task Management
**Context**: User wants to manage their personal tasks
- User authenticates via JWT
- User creates a new task with title, description, priority, tags, due date
- User retrieves all their tasks with filtering and sorting options
- User updates task details or marks as complete
- User deletes unwanted tasks

**Test**: System validates JWT, isolates user data, performs CRUD operations correctly

### Scenario 2: Advanced Task Features
**Context**: User wants to use advanced task management features
- User sets priority levels (high, medium, low) for tasks
- User assigns tags to organize tasks
- User sets due dates and recurrence patterns
- User searches across all task fields
- System handles recurring tasks by creating new instances when completed

**Test**: System properly stores, filters, sorts, and manages recurring tasks

### Scenario 3: Secure Access Control
**Context**: Multiple users accessing the system simultaneously
- User A creates and manages their tasks
- User B creates and manages their tasks
- User A cannot access User B's tasks and vice versa
- Unauthorized access attempts are rejected

**Test**: Data isolation is maintained, unauthorized access is prevented

## Functional Requirements

### FR-1: Authentication & Authorization
- The system shall verify JWT tokens from Authorization header using BETTER_AUTH_SECRET
- The system shall extract user_id from JWT payload and validate against URL parameter
- The system shall return HTTP 401 for invalid/missing tokens
- The system shall apply authentication to all /api/{user_id}/* routes

### FR-2: Task CRUD Operations
- The system shall allow creating tasks with title (required, 1-100 chars), description (≤5000 chars), priority, tags, due_date, recurrence
- The system shall retrieve all tasks for authenticated user with optional filtering and sorting
- The system shall update task properties with partial updates allowed
- The system shall delete tasks permanently (hard delete)
- The system shall toggle task completion status

### FR-3: Task Properties & Features
- The system shall support priority levels: high, medium, low
- The system shall support tagging with multiple tags per task
- The system shall support due dates stored in UTC and converted to user's local time for display
- The system shall support recurrence patterns: none, daily, weekly
- The system shall maintain created_at and updated_at timestamps

### FR-4: Search, Filter & Sort
- The system shall allow searching by keyword across title and description
- The system shall allow filtering by status (all, pending, completed)
- The system shall allow filtering by priority levels
- The system shall allow date range filtering (from/to specific dates)
- The system shall allow sorting by due_date, priority, or title in ascending/descending order

### FR-5: Recurring Tasks
- The system shall create new task instances when recurring tasks are marked complete
- The system shall calculate next due date based on recurrence pattern (daily: +1 day, weekly: +7 days)
- The system shall create recurring tasks with completed=False status
- The system shall handle recurrence asynchronously to avoid blocking requests
- The system shall create only the next instance when current task is completed (on-demand creation)

### FR-6: Data Isolation
- The system shall filter all queries by user_id to prevent cross-user access
- The system shall validate that requested resources belong to authenticated user
- The system shall return HTTP 404 for resources belonging to other users

## Non-Functional Requirements

### NFR-1: Performance
- The system shall respond to typical queries in under 200ms
- The system shall handle 1000+ tasks per user efficiently
- The system shall utilize database indexes for common query patterns

### NFR-2: Security
- The system shall validate all input data to prevent injection attacks
- The system shall enforce JWT authentication on all protected endpoints
- The system shall prevent data leakage between users
- The system shall implement rate limiting (100 requests per IP per hour) to prevent abuse

### NFR-3: Scalability
- The system shall leverage Neon Serverless PostgreSQL for automatic scaling
- The system shall use async operations where appropriate
- The system shall handle concurrent users efficiently

## Key Entities

### User
- Identity managed by Better Auth
- Properties: id (string, primary key), email (string, unique), name (string), created_at (timestamp)

### Task
- Properties: id (integer, primary key), user_id (foreign key to User.id), title (string, required), description (text, optional), completed (boolean, default False), priority (enum: high/medium/low), tags (array of strings), due_date (datetime, optional), recurrence (enum: none/daily/weekly), created_at (timestamp), updated_at (timestamp)

## Success Criteria

- 95% of API requests return responses in under 200ms
- System successfully isolates data between users with 100% accuracy
- All authenticated endpoints properly validate JWT tokens
- Recurring tasks are automatically created when completed with correct due dates
- Users can create, read, update, and delete their tasks without errors
- Search and filtering operations return accurate results within 500ms
- System handles 100 concurrent users without performance degradation

## Constraints

- No frontend UI or API client implementation
- Must rely on Better Auth for user management
- Use only specified tech stack (FastAPI, SQLModel, Neon PostgreSQL)
- Implementation via Claude Code following spec-driven approach
- No real-time features (websockets) required
- No advanced scheduling (cron jobs) for reminders

## Clarifications

### Session 2026-01-12

- Q: How should the backend handle excessive API requests? → A: Implement standard rate limiting (100 requests/IP/hour)
- Q: Should recurring tasks be created upfront or on-demand? → A: Create on-demand only (when current task is completed)
- Q: How should the system handle time zones for due dates? → A: Store in UTC and convert to user's local time for display
- Q: What should be the maximum length for task titles and descriptions? → A: Title: 100 chars max, Description: 5000 chars max
- Q: How should the system handle soft deletes versus hard deletes for tasks? → A: Hard delete by default, but log deletion events for audit purposes

## Assumptions

- Better Auth provides user management and JWT generation
- Frontend will consume the API endpoints as specified
- Neon Serverless PostgreSQL handles database scaling automatically
- Network connectivity between components is reliable
- JWT secret is securely shared between auth system and backend