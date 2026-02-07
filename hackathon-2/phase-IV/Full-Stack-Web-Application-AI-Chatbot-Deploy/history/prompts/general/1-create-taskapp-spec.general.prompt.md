---
id: 1
title: "create-taskapp-spec"
stage: "general"
date_iso: "2026-01-14"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "none"
branch: "3-full-stack-todo-app"
user: "user"
command: "sp.specify"
labels: ["specification", "todo-app", "full-stack"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/3-full-stack-todo-app/spec.md
  - specs/3-full-stack-todo-app/checklists/requirements.md
  - history/prompts/general/1-create-taskapp-spec.general.prompt.md
tests_yaml: |
  - "Specification created with all required sections"
  - "Quality checklist validated"
  - "Branch created successfully"
---

# Create TASKAPP Specification

## Prompt Text

# sp.specify
**Project**: TASKAPP – Professional Full-Stack Todo Application (Phase II – Final Edition)
**Branch**: feature/professional-styling-and-complete-integration
**Status**: Final & Actionable
**Word count**: 1100

## 1. Ultimate Goal (One-sentence mission)
Deliver a **premium, production-grade todo web app** with stunning public landing page, buttery-smooth dark/light theme, real JWT authentication, zero mocks, and full end-to-end task management — all running flawlessly with real Neon PostgreSQL data.

## 2. Non-negotiable Hard Rules (Violation = Immediate Failure)

1. **Zero mock/fake/dummy/placeholder data** anywhere (grep -ril "mock\|fake\|dummy\|placeholder\|hardcode" . → must be empty)
2. **Real authentication only** — Better Auth (frontend) ↔ PyJWT (backend) with shared `BETTER_AUTH_SECRET`
3. **No hydration mismatch** or **theme flicker** (use next-themes + correct SSR strategy)
4. **Build & lint gates**:
   - `npm run build` → 0 errors
   - `npm run lint` → 0 violations
   - `npm run dev` → no crash, no warnings
   - `uv run uvicorn backend.main:app` → no runtime warnings
5. All dates → real ISO/datetime, correctly persisted & displayed (user-local timezone)
6. Every "register" → replaced with **"Sign Up"** (UI, code, routes, titles)
7. No dead code, no commented logic, no TODOs, no console.log/debug

## 3. Visual Language & Design System (2026 Premium Standard)

**Palette**
- Light: bg #ffffff / #f8fafc, text #0f172a, primary #3b82f6, success #10b981, danger #ef4444
- Dark:  bg #020617 / #0f172a, text #e2e8f0, primary #60a5fa, success #34d399, danger #f87171

**Effects**
- Glassmorphism: `backdrop-filter: blur(16px)`, `background: rgba(255,255,255,0.06)` (dark) / `rgba(15,23,42,0.06)` (light)
- Gradients: `linear-gradient(135deg, #667eea 0%, #764ba2 100%)` for CTAs
- Shadows: subtle `shadow-xl` + hover lift
- Transitions: 300ms ease-out for all interactive elements

**Typography**
- Font: Inter (sans-serif), weights 400–700
- Headings: 4rem hero → 1.25rem cards
- Line height: 1.5–1.6

**Header** (fixed, glass effect)
- Left: **TASKAPP** (logo + bold text)
- Center (≥lg): Dashboard (only when logged in)
- Right: Theme toggle (sun/moon, animated spin) + Sign Up / Login (public) OR Avatar + Logout (authenticated)

**Hero**
- Headline: "Master Your Day with TASKAPP"
- Sub: "Beautiful. Intelligent. Truly yours."
- Primary CTA: Gradient "Sign Up – It's Free"
- Secondary: "Already using? Login"
- Background: animated subtle gradient mesh + floating glass cards

**Additional Sections** (≥3)
1. Features (4–6 glass cards: Priorities, Recurring, Smart Search, Notifications…)
2. How it Works (3-step timeline illustration)
3. Testimonials or Stats (minimal, elegant)
4. Final CTA footer

## 4. Routes & Access Control Matrix

| Path                | Access         | Core Content                                                                 | Redirect if unauthorized |
|---------------------|----------------|------------------------------------------------------------------------------|---------------------------|
| `/`                 | Public         | Landing + Hero + Sections + CTA                                              | —                         |
| `/sign-up`          | Public         | Name, Email, Password, Confirm + real-time validation                        | → /dashboard (success)    |
| `/login`            | Public         | Email, Password + forgot link (optional)                                     | → /dashboard (success)    |
| `/dashboard`        | Auth only      | Task list (real), add form, filters, search, sort, all CRUD + advanced      | → /login                  |
| `/tasks/:id`        | Auth only      | Task detail + edit                                                           | → /login                  |
| `*` (404)           | —              | Beautiful 404 page with home redirect                                        | —                         |

## 5. Theme Implementation Requirements

- Library: **next-themes** (recommended 2026 pattern)
- Strategy: `ThemeProvider` wrap + `useTheme()` hook
- Persistence: localStorage + `system` / `dark` / `light`
- No flicker: correct `suppressHydrationWarning` + initial class set via script
- Toggle: animated icon transition (rotate + scale)

## 6. Data & Integration Hard Rules

- **API Client** (`lib/api.ts`): real `fetch` + token from Better Auth session
- **No client-side storage** of tasks (except theme pref)
- **Recurring tasks**: real backend logic → new instance on complete
- **Reminders**: browser Notification API (request permission on set)
- **Search/Filter/Sort**: real query params → backend dynamic SQL

## 7. Quality & Acceptance Checklist (All Must Pass)

- [ ] Zero mock data (grep test)
- [ ] Full E2E flow: anonymous → sign up → dashboard → create recurring → complete → new task → logout → login → see data
- [ ] Theme change persists across pages/sessions
- [ ] No console errors/warnings (F12 during full usage)
- [ ] Build, lint, dev, uv run → all green
- [ ] Date display correct (e.g. "14 Jan 2026 • 14:30 PKT")
- [ ] Glassmorphism + gradients visible & beautiful in both themes
- [ ] Mobile: perfect stacking (hamburger menu if needed)

## 8. Execution Priority Order (Recommended Claude Code Sequence)

1. Root `.env` + uv workspace setup
2. Landing page + header + theme engine
3. Sign Up / Login pages (real auth)
4. Dashboard layout + real API connection (remove mocks)
5. Theme polish + glass/gradient effects
6. Full feature integration test (recurring, filters, etc.)
7. Final lint/build/zero-error gates

**Done =** A beautiful, professional, real-data todo app that feels like a premium SaaS product in 2026.

file with a dedicated, detailed, professional Database Schema section added. This incorporates PostgreSQL + SQLModel best practices (as of 2026), multi-user isolation, proper indexing, recurrence handling, and scalability considerations.
Markdown# sp.specify
**Project**: TASKAPP – Professional Full-Stack Todo Application (Phase II – Final Edition)
**Branch**: feature/professional-styling-and-complete-integration
**Status**: Final & Actionable
**Word count**: 1100

## 1. Ultimate Goal (One-sentence mission)
Deliver a **premium, production-grade todo web app** with stunning public landing page, buttery-smooth dark/light theme, real JWT authentication, zero mocks, and full end-to-end task management — all running flawlessly with real Neon PostgreSQL data.

## 2. Non-negotiable Hard Rules (Violation = Immediate Failure)

1. **Zero mock/fake/dummy/placeholder data** (grep -ril "mock\|fake\|dummy\|placeholder\|hardcode" . → must be empty)
2. **Real authentication only** — Better Auth (frontend) ↔ PyJWT (backend) with shared `BETTER_AUTH_SECRET`
3. **No hydration mismatch** or **theme flicker** (use next-themes + correct SSR strategy)
4. **Build & lint gates**:
   - `npm run build` → 0 errors
   - `npm run lint` → 0 violations
   - `npm run dev` → no crash, no warnings
   - `uv run uvicorn backend.main:app` → no runtime warnings
5. All dates → real ISO/datetime, correctly persisted & displayed (user-local timezone)
6. Every "register" → replaced with **"Sign Up"** (UI, code, routes, titles)
7. No dead code, no commented logic, no TODOs, no console.log/debug

## 3. Database Schema (PostgreSQL + SQLModel – 2026 Best Practices)

**Database**: Neon Serverless PostgreSQL
**ORM**: SQLModel (SQLAlchemy + Pydantic)
**Design Principles**:
- Multi-user isolation (every task belongs to exactly one user)
- Strong normalization (avoid redundancy)
- Proper indexing for performance (user_id, status, due_date, priority)
- Use `UUID` or `bigint` for IDs (future-proof scalability)
- Enforce constraints (NOT NULL, CHECK, UNIQUE where appropriate)
- Timestamps with timezone awareness (`timestamptz`)
- Array type for tags (PostgreSQL native `text[]`)
- Enum types for controlled values (priority, recurrence)

### Core Tables

1. **users** (managed by Better Auth – do NOT recreate manually)
   - `id`: `UUID` or `VARCHAR(36)` PRIMARY KEY
   - `email`: `VARCHAR(255)` UNIQUE NOT NULL
   - `name`: `VARCHAR(100)` NOT NULL
   - `created_at`: `timestamptz` DEFAULT NOW()
   - Indexes: UNIQUE(email)

2. **tasks** (main entity – owned by user)
   ```python
   from sqlmodel import SQLModel, Field
   from typing import Optional, List
   from datetime import datetime
   from enum import Enum

   class Priority(str, Enum):
       LOW = "low"
       MEDIUM = "medium"
       HIGH = "high"

   class Recurrence(str, Enum):
       NONE = "none"
       DAILY = "daily"
       WEEKLY = "weekly"

   class Task(SQLModel, table=True):
       id: Optional[int] = Field(default=None, primary_key=True)  # or UUIDField()
       user_id: str = Field(foreign_key="users.id", index=True)   # critical for isolation
       title: str = Field(max_length=200, nullable=False)
       description: Optional[str] = Field(max_length=1000, nullable=True)
       completed: bool = Field(default=False, index=True)
       priority: Priority = Field(default=Priority.MEDIUM)
       tags: List[str] = Field(sa_column=Column(ARRAY(String)), default_factory=list)
       due_date: Optional[datetime] = Field(sa_column=Column(DateTime(timezone=True)), nullable=True)
       recurrence: Recurrence = Field(default=Recurrence.NONE)
       created_at: datetime = Field(default_factory=datetime.utcnow, nullable=False)
       updated_at: datetime = Field(default_factory=datetime.utcnow, nullable=False)

       class Config:
           arbitrary_types_allowed = True
Indexes & Performance (Required)
SQLCREATE INDEX idx_tasks_user_id        ON tasks (user_id);
CREATE INDEX idx_tasks_completed      ON tasks (completed);
CREATE INDEX idx_tasks_due_date       ON tasks (due_date);
CREATE INDEX idx_tasks_priority       ON tasks (priority);
CREATE INDEX idx_tasks_user_completed ON tasks (user_id, completed);  -- composite for status filtering
-- Optional GIN for tag search (if full-text or array overlap needed)
CREATE INDEX idx_tasks_tags_gin       ON tasks USING GIN (tags);
Recurrence Logic Notes (Backend Responsibility)

When PATCH /tasks/{id}/complete and recurrence != NONE:
Update current task completed = True
Background task creates new task:
Copy most fields
Set new due_date = current due_date + delta (daily → +1 day, weekly → +7 days)
Reset completed = False
Limit future generations (e.g. max 50 instances per chain – store parent_id if needed)



Migration & Setup

Use Alembic (create backend/alembic/ if missing)
Initial migration: SQLModel.metadata.create_all(engine)
Enforce user_id filtering on every query

4. Visual Language & Design System (2026 Premium Standard)
Palette

Light: bg #ffffff/#f8fafc, text #0f172a, primary #3b82f6, success #10b981, danger #ef4444
Dark:  bg #020617/#0f172a, text #e2e8f0, primary #60a5fa, success #34d399, danger #f87171

Effects

Glassmorphism: backdrop-filter: blur(16px), semi-transparent bg
Gradients: linear-gradient(135deg, #667eea 0%, #764ba2 100%) CTAs
Transitions: 300–400ms ease for all interactive states

Header (fixed, glass)

Left: TASKAPP logo
Center (desktop): Dashboard (auth only)
Right: Theme toggle + Sign Up/Login OR Avatar/Logout

Hero & Sections

Headline: "Master Your Day with TASKAPP"
Sub: "Beautiful. Intelligent. Truly yours."
4+ glass cards: Priorities, Recurring, Smart Filters, Notifications
Final CTA footer

5. Routes & Access Control Matrix

Path,Access,Core Content,Redirect if unauthorized
/,Public,Landing + Hero + Sections + CTA,—
/sign-up,Public,"Name, Email, Password, Confirm + validation",→ /dashboard
/login,Public,"Email, Password",→ /dashboard
/dashboard,Auth only,"Real tasks, CRUD, filters, search, sort, advanced features",→ /login
/tasks/:id,Auth only,Task detail/edit,→ /login

PathAccessCore ContentRedirect if unauthorized/PublicLanding + Hero + Sections + CTA—/sign-upPublicName, Email, Password, Confirm + validation→ /dashboard/loginPublicEmail, Password→ /dashboard/dashboardAuth onlyReal tasks, CRUD, filters, search, sort, advanced features→ /login/tasks/:idAuth onlyTask detail/edit→ /login
6. Theme Implementation Requirements

next-themes + useTheme()
localStorage + system preference
No flicker: script-based initial class + suppressHydrationWarning

7. Quality & Acceptance Checklist (All Must Pass)

 Zero mock data
 Full E2E: sign up → create recurring → complete → new task → logout/login → data persists
 Theme persists across sessions
 No console errors/warnings
 All build/lint/uv gates green
 Dates correct (e.g. "14 Jan 2026 • 14:30 PKT")
 Mobile responsive + glassmorphism visible in both themes

8. Execution Priority Order

Database schema + indexes + migration setup
Root .env + uv workspace
Landing + header + theme system
Sign Up/Login (real auth)
Dashboard + real API (remove mocks)
Full feature polish + glass/gradient effects
Final quality gates

Done = A beautiful, secure, real-data todo app that feels like a premium 2026 SaaS product.

2sFast30 sourcesAdd Alembic migration scriptsHere is the updated and enhanced version of the sp.specify file with a new dedicated section for Alembic Migration Scripts added.
It includes practical, production-ready guidance tailored for your monorepo + backend/ folder + SQLModel + uv setup (as of 2026 standards).
Markdown# sp.specify
**Project**: TASKAPP – Professional Full-Stack Todo Application (Phase II – Final Edition)
**Branch**: feature/professional-styling-and-complete-integration
**Status**: Final & Actionable
**Word count**: 1100

## 1. Ultimate Goal (One-sentence mission)
Deliver a **premium, production-grade todo web app** with stunning public landing page, buttery-smooth dark/light theme, real JWT authentication, zero mocks, and full end-to-end task management — all running flawlessly with real Neon PostgreSQL data **and version-controlled schema changes**.

## 2. Non-negotiable Hard Rules (Violation = Immediate Failure)

1. **Zero mock/fake/dummy/placeholder data** (grep -ril "mock\|fake\|dummy\|placeholder\|hardcode" . → must be empty)
2. **Real authentication only** — Better Auth ↔ PyJWT with shared `BETTER_AUTH_SECRET`
3. **No hydration mismatch** or **theme flicker** (next-themes + correct SSR)
4. **Build & lint gates**:
   - `npm run build` → 0 errors
   - `npm run lint` → 0 violations
   - `npm run dev` → no crash/warnings
   - `uv run uvicorn backend.main:app` → no runtime warnings
5. All dates → real ISO/datetime, persisted & displayed correctly (user-local timezone)
6. Every "register" → replaced with **"Sign Up"**
7. No dead code, no commented logic, no TODOs, no console.log/debug

## 3. Database Schema (PostgreSQL + SQLModel – 2026 Best Practices)

**Database**: Neon Serverless PostgreSQL
**ORM**: SQLModel
**Principles**: Multi-user isolation, strong normalization, performance indexing, timezone-aware timestamps, native PostgreSQL types

### Core Tables

1. **users** (managed by Better Auth – do not recreate)
   - `id`: `VARCHAR(36)` or `UUID` PRIMARY KEY
   - `email`: `VARCHAR(255)` UNIQUE NOT NULL
   - `name`: `VARCHAR(100)` NOT NULL
   - `created_at`: `timestamptz` DEFAULT NOW()

2. **tasks** (user-owned)
   ```python
   from sqlmodel import SQLModel, Field, Column
   from typing import Optional, List
   from datetime import datetime
   from enum import Enum
   from sqlalchemy.dialects.postgresql import ARRAY
   from sqlalchemy import String

   class Priority(str, Enum):
       LOW = "low"
       MEDIUM = "medium"
       HIGH = "high"

   class Recurrence(str, Enum):
       NONE = "none"
       DAILY = "daily"
       WEEKLY = "weekly"

   class Task(SQLModel, table=True):
       __tablename__ = "tasks"
       id: Optional[int] = Field(default=None, primary_key=True)  # or UUIDField()
       user_id: str = Field(foreign_key="users.id", index=True)
       title: str = Field(max_length=200, nullable=False)
       description: Optional[str] = Field(max_length=1000, nullable=True)
       completed: bool = Field(default=False, index=True)
       priority: Priority = Field(default=Priority.MEDIUM)
       tags: List[str] = Field(sa_column=Column(ARRAY(String)), default_factory=list)
       due_date: Optional[datetime] = Field(sa_column=Column(DateTime(timezone=True)), nullable=True)
       recurrence: Recurrence = Field(default=Recurrence.NONE)
       created_at: datetime = Field(default_factory=datetime.utcnow, nullable=False)
       updated_at: datetime = Field(
           default_factory=datetime.utcnow,
           sa_column_kwargs={"onupdate": datetime.utcnow},
           nullable=False
       )
Required Indexes
SQLCREATE INDEX idx_tasks_user_id          ON tasks (user_id);
CREATE INDEX idx_tasks_completed        ON tasks (completed);
CREATE INDEX idx_tasks_due_date         ON tasks (due_date);
CREATE INDEX idx_tasks_priority         ON tasks (priority);
CREATE INDEX idx_tasks_user_completed   ON tasks (user_id, completed);  -- composite
CREATE INDEX idx_tasks_tags_gin         ON tasks USING GIN (tags);      -- for tag search/filter
4. Alembic Migration Scripts (Versioned Schema Management)
Folder Location: backend/alembic/ (create if missing)
Purpose: Safely evolve schema over time (add fields, change types, add indexes, etc.)
Tool: Alembic 1.13+ (uv add alembic)
Setup Steps (Execute exactly once)

From backend/ folder:Bashuv add alembic
alembic init alembicResulting structure:textbackend/
├── alembic/
│   ├── env.py
│   ├── script.py.mako
│   └── versions/          ← migration scripts go here
├── alembic.ini            ← config file
└── ... (other files)
alembic.ini configuration (critical lines):ini[alembic]
script_location = alembic
sqlalchemy.url = postgresql+asyncpg://...  ; ← use env var in env.py instead
backend/alembic/env.py – Key modificationsPythonimport os
from dotenv import load_dotenv
from sqlmodel import SQLModel
from pathlib import Path
from logging.config import fileConfig

load_dotenv()  # loads root .env

# This is the Alembic Config object
from alembic import context
config = context.config

# Override sqlalchemy.url from environment
config.set_main_option("sqlalchemy.url", os.getenv("DATABASE_URL"))

# Import all models so metadata is populated
from backend.models import Task  # ← import ALL your models here!

target_metadata = SQLModel.metadata

# ... rest of file remains mostly default ...
backend/alembic/script.py.mako – Add this line near top imports:makoimport sqlmodel

Workflow (How to Create & Apply Migrations)

Change model (e.g. add new field category: str to Task)
Generate candidate migration:Bashcd backend
uv run alembic revision --autogenerate -m "add category field"→ creates alembic/versions/xxxx_add_category_field.py
Review & edit the generated file (autogenerate is helpful but never 100% perfect):
Check upgrade() and downgrade()
Add data migrations if needed (e.g. populate new column)
Fix any weird types (e.g. replace AutoString → String)

Apply migration:Bashuv run alembic upgrade head
Downgrade (if needed):Bashuv run alembic downgrade -1

Best Practices & Tips (2026)

Always review autogenerated scripts — especially column types, constraints, indexes
Commit migration files to git (they are code!)
Use revision IDs in branches (merge conflicts rare but possible)
Run alembic current to see applied version
For production: never SQLModel.metadata.create_all() — use only Alembic
First migration: usually alembic revision --autogenerate -m "initial schema"

5. Visual Language & Design System (unchanged – see previous version)
[...]
6. Routes & Access Control Matrix (unchanged)
[...]
7. Theme Implementation Requirements (unchanged)
[...]
8. Quality & Acceptance Checklist (All Must Pass)

 Alembic migrations created & applied successfully
alembic current shows correct head after setup
 Schema changes (e.g. add field) → generate, review, apply → app still works
 Zero mock data + all other previous gates

9. Execution Priority Order (Updated)

Database schema + indexes (models.py)
Alembic setup (alembic init, env.py, script.py.mako fixes)
Create initial migration & apply
Root .env + uv workspace
Landing + header + theme
Auth + dashboard + real API
Full polish + final quality gates

Done

## Response Text

I have successfully created the specification for the TASKAPP project:

✅ **Branch Created**: `3-full-stack-todo-app`

✅ **Spec File Created**: `specs/3-full-stack-todo-app/spec.md`

✅ **Quality Checklist Created**: `specs/3-full-stack-todo-app/checklists/requirements.md`

The specification includes:
- Comprehensive user scenarios with priorities (P1-P2)
- Detailed functional requirements (FR-001 to FR-015)
- Key entities definition (User and Task)
- Measurable success criteria (SC-001 to SC-010)
- Edge cases consideration
- Quality validation checklist

The specification follows all requirements:
- No implementation details leaked into the spec
- Technology-agnostic success criteria
- Testable and unambiguous requirements
- All mandatory sections completed
- No remaining [NEEDS CLARIFICATION] markers

The specification is now ready for the next phase (`/sp.clarify` or `/sp.plan`) and meets all quality standards defined in the checklist.

## Next Steps

The feature specification is complete and ready for:
1. Clarification of any remaining questions if needed (`/sp.clarify`)
2. Moving to planning phase (`/sp.plan`)
3. Implementation tasks generation (`/sp.tasks`)

The spec adheres to the project's hard rules including zero mock data, real authentication, no hydration mismatch, and all build/lint requirements.