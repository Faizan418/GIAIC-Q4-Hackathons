# Feature Specification: Full-Stack Todo App Integration

**Feature Branch**: `1-full-stack-integration`
**Created**: 2026-01-12
**Status**: Draft
**Input**: User description: "# Integration Specification for Phase II: Full-Stack Todo App (Frontend ↔ Backend Connection & Final Polish)

## Project Overview
This specification covers the **final integration phase** after frontend and backend are individually complete.
Objective:
- Connect live backend to frontend (remove all mocks/hardcoded data)
- Show only real data from PostgreSQL via FastAPI
- Fix every import error, comma error, type error, runtime error
- Ensure clean folder separation: frontend/ only contains frontend files, backend/ only contains backend files
- Create any missing files/folders
- Build backend exclusively with **uv** (https://docs.astral.sh/uv/) – no pip, no poetry, no venv+pip
- Final app must run perfectly with real authentication, real tasks, full features (basic + intermediate + advanced)

Target: Zero errors, production-ready local setup.

## Success Criteria
- [ ] Frontend shows real tasks from Neon DB (no mock data anywhere)
- [ ] JWT auth works end-to-end (login → protected routes → API calls succeed)
- [ ] All features work: create, complete, delete, update, priority, tags, due date, recurrence, search, filter, sort, reminders
- [ ] Zero console errors (frontend or backend)
- [ ] tsc --noEmit passes (frontend) + mypy --strict passes (backend)
- [ ] Backend created and runs with uv only
- [ ] docker-compose up works (or uv + next dev separately)

## Root Directory Final Structure (must be exactly this)

```
Full-Stack-Web-Application/                           ← monorepo ROOT
├── .env                                  ← SINGLE source of truth for ALL secrets
├── .env.example
├── .gitignore
├── docker-compose.yml                    ← optional dev setup (backend + frontend + watch)
├── uv.lock                               ← ★ Generated & committed (uv workspace lockfile)
├── pyproject.toml                        ← Root workspace config (or backend-specific)
├── README.md                             ← Updated with uv commands
├── specs/                                ← All previous specifications
├── frontend/                             ← Next.js 16+ App (no changes to structure)
│   ├── app/
│   ├── components/
│   ├── lib/api.ts                        ← Real backend calls only
│   ├── public/
│   ├── pages/
│   │   ├── index.js                     ← Main page
│   │   └── _app.js                      ← App wrapper
│   ├── types/
│   ├── styles/
│   │   └── globals.css                  ← Tailwind styles
│   ├──.env                              ← Environment variables (API keys)
│   ├── next.config.js
│   ├── tailwind.config.js
│   ├── package.json
│   └── tsconfig.json
└── backend/                              ← FastAPI + SQLModel (uv managed)
    ├── pyproject.toml                    ← Preferred (modern uv style)
    ├── main.py
    ├── models.py
    ├── schemas.py
    ├── crud/tasks.py
    ├── routes/tasks.py
    ├── db.py
    ├── auth.py                           ← JWT verification from root .env
    ├── utils.py                          ← Recurrence & helpers
    ├── alembic/                          ← Create if missing (migrations)
    ├── tests/                            ← Create if missing (pytest basics)
    ├── CLAUDE.md                         ← uv-specific instructions
    └── .env                              ← Environment variables (API keys)
```

## Mandatory File Checks & Fixes (read every file)

### Frontend – Must Fix These Exact Issues
1. lib/api.ts
   - Remove all mock data, mock fetch, hardcoded tasks
   - Use real fetch with JWT from Better Auth
   - Final code must be:

```ts
import { getSession } from "@better-auth/react";

const API_BASE = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000";

export const api = {
  async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    const session = await getSession();
    const token = session?.user?.accessToken;

    const res = await fetch(`${API_BASE}${endpoint}`, {
      ...options,
      headers: {
        "Content-Type": "application/json",
        Authorization: token ? `Bearer ${token}` : "",
        ...options.headers,
      },
    });

    if (!res.ok) {
      const error = await res.json().catch(() => ({}));
      throw new Error(error.detail || res.statusText);
    }

    return res.json();
  },

  getTasks: (userId: string, params = "") =>
    api.request<Task[]>(`/api/${userId}/tasks${params}`),
  createTask: (userId: string, data: CreateTask) =>
    api.request<Task>(`/api/${userId}/tasks`, { method: "POST", body: JSON.stringify(data) }),
  // ... all other methods (update, delete, toggleComplete, etc.)
};
```

2. All components & pages
   - Replace any dummy tasks with real data from useSWR or fetch in server components
   - Remove any `const mockTasks = [...]`
   - Ensure userId comes from Better Auth session (not hardcoded)

3. Better Auth config
   - Must enable JWT plugin and use same secret as backend

```ts
// auth.ts or better-auth config
import { betterAuth } from "better-auth";
export const auth = betterAuth({
  plugins: [jwtPlugin({ secret: process.env.BETTER_AUTH_SECRET! })],
});
```

### Backend – Must Be Built With uv Only

**Step-by-step uv setup (must follow exactly):**

```bash
# From monorepo root
uv init
uv add fastapi uvicorn sqlmodel psycopg2-binary pyjwt python-dotenv pydantic[email]

# pyproject.toml will be created/updated with:
[project]
dependencies = [
  "fastapi",
  "uvicorn",
  "sqlmodel",
  "psycopg2-binary",
  "pyjwt",
  "python-dotenv",
  "pydantic[email]",
]
```

Run backend with uv:

```bash
uv run uvicorn backend.main:app --reload --port 8000
```

**Critical backend fixes:**

1. auth.py – Final JWT verification

```python
from fastapi import Depends, HTTPException, Header
import jwt
import os

SECRET = os.getenv("BETTER_AUTH_SECRET")

def get_current_user(authorization: str = Header(None)):
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(401, "Missing token")
    token = authorization.split(" ")[1]
    try:
        payload = jwt.decode(token, SECRET, algorithms=["HS256"])
        user_id: str = payload.get("userId")
        if not user_id:
            raise HTTPException(401, "Invalid token")
        return user_id
    except jwt.PyJWTError:
        raise HTTPException(401, "Invalid token")
```

2. All routes – Must use user_id from token AND path

```python
@router.get("/tasks")
async def get_tasks(user_id: str = Depends(get_current_user), current_path_id: str = Path(...)):
    if user_id != current_path_id:
        raise HTTPException(403, "Forbidden")
    # then query with user_id
```

3. db.py – Async engine with root .env

```python
from sqlmodel import SQLModel, create_engine
from dotenv import load_dotenv
import os

load_dotenv()  # loads root .env
DATABASE_URL = os.getenv("DATABASE_URL")
engine = create_engine(DATABASE_URL, echo=True, future=True)
```

4. main.py – Include router with prefix

```python
app.include_router(tasks_router, prefix="/api/{user_id}")
```

## Missing Folders/Files to Create
- backend/alembic/ + versions/ (empty for now)
- backend/tests/__init__.py + test_tasks.py (basic pytest)
- frontend/lib/auth.ts (if missing)

## Final Connection Steps (Claude must execute exactly)
1. Start backend: `uv run uvicorn backend.main:app --reload`
2. Start frontend: `cd frontend && npm run dev`
3. Open http://localhost:3000
4. Register → login → create task → refresh → task persists and reappears
5. Test recurrence: complete recurring task → new instance appears
6. Test filters/sort/search → all work with real data

## Acceptance Criteria
- Real Neon DB has tasks after creation
- No mock data in any file (grep -r "mock" must re"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Connect Frontend to Backend API (Priority: P1)

User wants to see real tasks from the database in the frontend instead of mock data. When the user logs in, creates tasks, or performs any task operations, the data should persist in the PostgreSQL database and be retrieved through the FastAPI backend.

**Why this priority**: This is the core functionality that connects the frontend and backend systems, enabling the entire application to function as a unified system rather than separate components.

**Independent Test**: Can be fully tested by logging into the application, creating a task, refreshing the page, and verifying the task still exists. This delivers the fundamental value of persistent task management.

**Acceptance Scenarios**:

1. **Given** user is logged in and connected to the backend API, **When** user creates a new task, **Then** the task is saved to the PostgreSQL database and appears in the task list
2. **Given** user has created tasks in the system, **When** user refreshes the page, **Then** the tasks are retrieved from the database and displayed
3. **Given** user has tasks in the database, **When** user updates a task, **Then** the changes are persisted to the database

---

### User Story 2 - End-to-End Authentication Flow (Priority: P1)

User wants to log in once and have their authentication token automatically used for all API calls to the backend. The JWT token from Better Auth must be properly attached to all requests to the backend API.

**Why this priority**: Authentication is critical for security and user data isolation. Without proper authentication, users cannot access their personal data or perform authenticated operations.

**Independent Test**: Can be fully tested by logging in, then making API calls to the backend and verifying that the JWT token is properly validated and the user can only access their own data.

**Acceptance Scenarios**:

1. **Given** user has successfully logged in, **When** user performs any API operation, **Then** the JWT token is automatically included in the request headers
2. **Given** user has a valid JWT token, **When** user makes a request to the backend, **Then** the token is validated and user data is properly isolated
3. **Given** user has an invalid or expired JWT token, **When** user makes a request to the backend, **Then** the request is rejected with appropriate error response

---

### User Story 3 - Full Feature Integration (Priority: P2)

User wants all todo features to work end-to-end: create, complete, delete, update, priority, tags, due date, recurrence, search, filter, sort. All functionality previously developed in isolation should now work with real data.

**Why this priority**: This ensures the complete user experience is delivered, allowing users to manage their tasks with all advanced features available.

**Independent Test**: Can be fully tested by using each feature individually with real data and verifying proper persistence and retrieval from the database.

**Acceptance Scenarios**:

1. **Given** user has created recurring tasks, **When** user completes a recurring task, **Then** a new instance of the task is created in the database for the next occurrence
2. **Given** user has multiple tasks with different priorities, tags, and due dates, **When** user applies filters or sorting, **Then** the tasks are properly filtered and sorted based on the criteria
3. **Given** user has many tasks in the system, **When** user searches for specific tasks, **Then** the search returns relevant results from the database

---

### User Story 4 - Development Environment Setup (Priority: P1)

User wants to be able to run the complete application with uv for backend dependency management and proper tooling setup, ensuring consistent development environment across all team members.

**Why this priority**: Proper development environment setup is critical for reliable development, testing, and deployment of the application.

**Independent Test**: Can be fully tested by running the application with uv and verifying all dependencies are properly managed and the application starts successfully.

**Acceptance Scenarios**:

1. **Given** development environment with uv installed, **When** user runs uv commands to set up the backend, **Then** all dependencies are properly installed and managed
2. **Given** backend dependencies defined in pyproject.toml, **When** user runs the backend server, **Then** the application starts without dependency errors
3. **Given** frontend and backend configured properly, **When** user starts both services, **Then** they communicate correctly with zero console errors

---

### Edge Cases

- What happens when the database is temporarily unavailable during API requests?
- How does the system handle JWT token expiration during long-running operations?
- What happens when the frontend and backend are running on different ports and CORS restrictions apply?
- How does the system handle large numbers of concurrent users accessing the API?
- What happens when the environment variables (secrets) are not properly configured?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST connect the frontend to the backend API using real data instead of mock data
- **FR-002**: System MUST authenticate users with JWT tokens from Better Auth and validate them on the backend
- **FR-003**: System MUST ensure user data isolation so users can only access their own tasks
- **FR-004**: System MUST persist all task operations (create, update, delete, complete) to PostgreSQL database
- **FR-005**: System MUST support all todo features: priorities, tags, due dates, recurrence, search, filter, sort
- **FR-006**: System MUST handle recurring tasks by creating new instances when completed
- **FR-007**: System MUST provide proper error handling for API failures and network issues
- **FR-008**: System MUST validate all API requests and responses with proper authentication
- **FR-009**: System MUST use uv for backend dependency management instead of pip
- **FR-010**: System MUST run without console errors on both frontend and backend
- **FR-011**: System MUST support proper folder separation with frontend/ and backend/ directories
- **FR-012**: System MUST load configuration from environment variables (BETTER_AUTH_SECRET, DATABASE_URL)

### Key Entities *(include if feature involves data)*

- **User**: Represents an authenticated user with unique ID, manages their own tasks
- **Task**: Represents a todo item with title, description, priority, tags, due date, recurrence pattern, completion status
- **JWT Token**: Authentication token that provides access to user-specific data and ensures data isolation
- **API Session**: Represents the connection between frontend and backend with proper authentication headers

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Frontend shows real tasks from PostgreSQL database with no mock data present (verify with grep -r "mock" returns no results in any files)
- **SC-002**: JWT authentication works end-to-end with login, protected routes, and successful API calls using tokens
- **SC-003**: All todo features work correctly: create, complete, delete, update, priority, tags, due date, recurrence, search, filter, sort
- **SC-004**: Zero console errors appear in either frontend or backend during normal operation
- **SC-005**: Backend runs successfully with uv dependency management and passes tsc --noEmit (frontend) and mypy --strict (backend) checks
- **SC-006**: Application can be started with docker-compose up or with uv + next dev separately and runs without errors
- **SC-007**: Recurring tasks properly create new instances when completed and appear in the task list
- **SC-008**: User data isolation is maintained with users only able to access their own tasks