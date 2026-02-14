---
id: 9
title: Backend Todo API Plan
stage: general
date: 2026-01-12
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: unknown
command: sp.plan
labels: ["backend", "todo-api", "fastapi", "planning"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - E:\Phase 2\Full-Stack-Web-Application\specs\1-backend-todo-api\plan.md
 - E:\Phase 2\Full-Stack-Web-Application\specs\1-backend-todo-api\research.md
 - E:\Phase 2\Full-Stack-Web-Application\specs\1-backend-todo-api\data-model.md
 - E:\Phase 2\Full-Stack-Web-Application\specs\1-backend-todo-api\contracts\api-contracts.md
 - E:\Phase 2\Full-Stack-Web-Application\specs\1-backend-todo-api\quickstart.md
 - E:\Phase 2\Full-Stack-Web-Application\backend\CLAUDE.md
 - E:\Phase 2\Full-Stack-Web-Application\backend\requirements.txt
tests:
 - none
---

## Prompt

# sp.plan
**Project: Backend Implementation for Phase II Todo Web Application**
**Phase: Backend-only development (frontend already completed)**
**Current Date: January 11, 2026**
**Development Paradigm: Agentic + Spec-Driven (Claude Code + Spec-Kit Plus)**
**Goal of this plan: Place all credentials & secrets management in the root directory level (monorepo root)**

## 1. High-Level Architecture Sketch (Backend Focus)

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
│   ├── routes/
│   ├── db.py
│   ├── auth.py                        ← reads BETTER_AUTH_SECRET from os.environ
│   ├── utils.py
│   ├── requirements.txt
│   ├── CLAUDE.md
│   └── alembic/                       ← migrations (optional future)
└── README.md
```

**Central Credentials Strategy**
All sensitive values are stored **only once** in the root `.env` file.
Both frontend and backend (and docker-compose) load secrets from the **same root location**.

## 2. Root-Level Credentials Management (Decision Table)

| # | Credential                  | Stored In          | How Accessed                              | Who Needs It                          | Security Notes                              |
|---|-----------------------------|--------------------|-------------------------------------------|---------------------------------------|---------------------------------------------|
| 1 | BETTER_AUTH_SECRET          | root `.env`        | `os.getenv("BETTER_AUTH_SECRET")`         | Frontend + Backend (JWT sign/verify)  | Must be identical in both services          |
| 2 | DATABASE_URL                | root `.env`        | `os.getenv("DATABASE_URL")`               | Backend only (Neon PostgreSQL)        | Connection string with password             |
| 3 | NEON_API_KEY                | root `.env`        | (optional – direct Neon SDK if used)      | (Future admin scripts)                | Rarely needed in runtime                    |
| 4 | JWT_ALGORITHM               | root `.env`        | `os.getenv("JWT_ALGORITHM", "HS256")`     | Backend (verification)                | Usually fixed, but configurable             |
| 5 | ACCESS_TOKEN_EXPIRE_MINUTES | root `.env`        | Default 10080 (7 days) if not set         | Backend (if token expiry enforced)    | Better Auth controls, but can override      |

**.env.example** (committed to git):

```bash
# .env.example - copy to .env and fill real values

# ── Authentication (must be same for frontend & backend)
BETTER_AUTH_SECRET=your-very-long-random-secret-here

# ── Database (Neon serverless PostgreSQL)
DATABASE_URL=postgresql+asyncpg://user:password@ep-project-name-123456.us-east-2.aws.neon.tech/dbname?sslmode=require

# Optional overrides / development settings
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=10080
```

## 3. Important Architectural & Security Decisions

| # | Decision                                | Options Considered                          | Chosen                              | Tradeoffs / Rationale                                                                 |
|---|-----------------------------------------|---------------------------------------------|-------------------------------------|---------------------------------------------------------------------------------------|
| 1 | Where to store secrets                  | Root `.env` / backend/.env / docker secrets | **Root `.env`**                     | Single source of truth, easy for monorepo, works with docker-compose env_file         |
| 2 | How frontend accesses root .env         | Symlink / copy / read from parent dir       | **Vite/Next.js env loading**        | Next.js automatically loads `.env` from project root when running from monorepo root  |
| 3 | JWT verification library                | PyJWT / python-jose / fastapi-jwt-auth      | **PyJWT** (simple & fast)           | Minimal dependencies, full control over verification process                          |
| 4 | Dependency injection for user           | Global middleware / per-route dependency    | **FastAPI Depends + middleware**    | Middleware for 401 early exit, Depends for type-safe current_user                     |
| 5 | Database session management             | Global engine / per-request session         | **Per-request via Depends**         | Thread-safety, proper transaction scope, works with async                             |
| 6 | Environment variable loading            | python-dotenv / os.environ direct           | **python-dotenv + os.getenv**       | Safe fallback, explicit loading in development                                        |

## 4. Agent Structure & Responsibilities
**@agents mansions** – backend-focused agents (document in backend/CLAUDE.md)

| Agent Name                     | Primary Responsibility                                      | Preferred Spec References                     | Output Style                        |
|--------------------------------|-------------------------------------------------------------|-----------------------------------------------|-------------------------------------|
| claude/db-architect            | Models, relationships, indexes, migration strategy          | @specs/database/schema.md                     | SQLModel-first                      |
| claude/api-designer            | Endpoint logic, query building, response shaping            | @specs/api/rest-endpoints.md                  | OpenAPI-friendly                    |
| claude/auth-enforcer           | JWT middleware, user dependency, ownership checks           | @specs/features/authentication.md             | Security checklist style            |
| claude/query-optimizer         | Dynamic filters, sorting, pagination, indexing advice       | Task filtering & sorting requirements         | SQL performance focused             |
| claude/recurrence-engineer     | Recurring task logic, next occurrence calculation           | Advanced features spec                        | Algorithm + edge-case list          |
| claude/feature-breaker         | Breaks implementation into smallest safe tasks               | All backend specs                             | Numbered, prioritized tasks         |

## 5. Development Sequencing (recommended order)

1. Root credentials setup (.env + .env.example)
2. Database connection & basic models (db.py + models.py)
3. JWT middleware & current_user dependency (auth.py)
4. Basic CRUD endpoints skeleton (routes/tasks.py)
5. Full basic level implementation (create/read/update/delete/toggle)
6. Intermediate: dynamic query filters, search, sort, priority, tags
7. Advanced: due_date handling, recurrence logic on complete
8. Error handling, validation, HTTP responses standardization
9. Testing & documentation (Swagger + pytest)

## 6. Quality & Security Validation Strategy

**Layered checks** (execution order):

1. **Static checks**
   - mypy --strict
   - ruff check + ruff format

2. **Secret leakage prevention**
   - git-secrets or truffleHog scan
   - .env never committed (enforced by .gitignore)

3. **Unit / Integration Tests** (pytest)

   - Test JWT verification (valid/invalid/expired/missing)
   - Test ownership enforcement (user A cannot see user B)
   - Test all CRUD paths + filters
   - Test recurrence clone creation

4. **Security Checklist** (manual review)

   - [ ] All endpoints require JWT
   - [ ] user_id in path must match token user_id
   - [ ] No raw SQL (only SQLModel)
   - [ ] Input val

## Response snapshot

Generated comprehensive backend implementation plan for Todo API including architecture, research, data models, API contracts, quickstart guide, and development guidelines.

## Outcome

- ✅ Impact: Established complete backend architecture and implementation roadmap for Todo API with proper authentication, data models, and API contracts
- 🧪 Tests: none
- 📁 Files: Created plan.md, research.md, data-model.md, api-contracts.md, quickstart.md, backend/CLAUDE.md, backend/requirements.txt
- 🔁 Next prompts: /sp.tasks to break implementation into executable tasks
- 🧠 Reflection: Plan provides complete foundation for backend development with security-first approach and proper user data isolation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate implementation tasks based on this plan