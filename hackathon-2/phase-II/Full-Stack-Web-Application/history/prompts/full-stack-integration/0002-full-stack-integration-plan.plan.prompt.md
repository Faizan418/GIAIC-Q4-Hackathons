---
id: "PHR-0002"
title: "Full-Stack Integration Plan"
stage: "plan"
date: "2026-01-12"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "full-stack-integration"
branch: "1-full-stack-integration"
user: "unknown"
command: "/sp.plan"
labels: ["integration", "full-stack", "todo-app", "planning"]
links:
  spec: "specs/1-full-stack-integration/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/1-full-stack-integration/plan.md"
  - "specs/1-full-stack-integration/research.md"
  - "specs/1-full-stack-integration/data-model.md"
  - "specs/1-full-stack-integration/contracts/openapi-contract.yaml"
  - "specs/1-full-stack-integration/quickstart.md"
  - "specs/1-full-stack-integration/agent-context-update.md"
tests: []
---

# COMMAND: sp.plan

## CONTEXT

Created comprehensive implementation plan for the full-stack todo app integration feature, connecting frontend and backend systems with real data flow.

## PROMPT_TEXT

# sp.plan
**Project: Final Integration & Production-Ready Phase II Todo Full-Stack Application**
**Phase: Post-completion polish, full connection, error elimination & uv standardization**
**Current Date: January 12, 2026**
**Development Paradigm: Agentic + Spec-Driven (Claude Code + Spec-Kit Plus)**
**Core Goal**: Achieve zero-error, real-data-only, fully connected app with backend exclusively managed via **uv** (Astral's modern Python tool)

## 1. High-Level Architecture Sketch (Final State)

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

## 2. uv Standardization Decisions (2026 Best Practices)

| # | Decision                              | Options Considered                       | Chosen Approach                          | Tradeoffs / Rationale (Jan 2026)                          |
|---|---------------------------------------|------------------------------------------|------------------------------------------|------------------------------------------------------------|
| 1 | Dependency Management Tool            | pip + venv / poetry / pdm / uv           | **uv only** (no pip fallback)            | Fastest resolver/install, modern monorepo support, Rust speed |
| 2 | Project Layout                        | Single pyproject.toml / workspace        | **Workspace mode** (backend as member)   | Better for future scaling, clean separation               |
| 3 | Lockfile Location                     | Root / backend/                          | **Root uv.lock**                         | Single lock for reproducibility, standard in 2025–2026    |
| 4 | Run Commands                          | uvicorn direct / poetry run / uv run     | **uv run uvicorn ...**                   | Zero venv activation needed, consistent & fast             |
| 5 | Docker Integration                    | Multi-stage classic / uv layers          | uv multi-stage + COPY uv.lock            | Excellent caching, smaller images, official recommendation|

**Recommended root pyproject.toml** (minimal workspace style):

```toml
[project]
name = "todo-monorepo"
version = "0.1.0"
requires-python = ">=3.10"

[tool.uv]
# Enables workspace mode
sources = { backend = { path = "backend", editable = true } }

[tool.uv.workspace]
members = ["backend"]
```

(Alternative: put full pyproject.toml only inside backend/ — both work in 2026)

## 3. Critical Integration & Error Elimination Checklist

| Category                  | Specific Checks / Fixes to Execute                                                                 | Priority | Owner Agent Suggestion     |
|---------------------------|----------------------------------------------------------------------------------------------------|----------|----------------------------|
| Secrets                   | All in root `.env` only; `load_dotenv()` in db.py & auth.py; frontend uses `process.env`         | ★★★★★    | auth-enforcer              |
| Mock/Hardcoded Data       | Grep entire codebase for `mock`, `hardcode`, dummy arrays → remove all                           | ★★★★★    | data-flow                  |
| JWT Flow                  | Frontend → Better Auth JWT → header → backend PyJWT verify → user_id match path                  | ★★★★     | auth-enforcer              |
| API Client                | `lib/api.ts` uses real `fetch` with token; base URL from env (NEXT_PUBLIC_API_URL)               | ★★★★     | ui-architect               |
| Missing Folders/Files     | Create: backend/alembic/, backend/tests/, backend/pyproject.toml if absent                       | ★★★      | feature-breaker            |
| Import & Syntax Errors    | Run `tsc --noEmit` (frontend) + `uv run mypy .` + `uv run ruff check` (backend) → fix all       | ★★★★★    | query-optimizer            |
| Recurring Logic           | PATCH /complete → if recurring → background task creates next instance                           | ★★★      | recurrence-engineer        |
| Real Data Validation      | After login → create task → logout → login again → task still there (Neon DB persistence)       | ★★★★★    | data-flow                  |
| Performance & Indexing    | Ensure indexes on tasks: user_id, completed, due_date, priority                                  | ★★★      | db-architect               |

## 4. Agent Structure & Responsibilities (Updated for Final Phase)

| Agent Name                  | Primary Responsibility (Final Phase)                           | Focus Area                     |
|-----------------------------|----------------------------------------------------------------|--------------------------------|
| claude/auth-enforcer        | End-to-end JWT roundtrip, 401/403 handling                     | Security & isolation           |
| claude/data-flow            | Remove mocks, ensure real DB ↔ UI data flow                    | Real data only                 |
| claude/db-architect         | Indexes, session management, alembic init if needed            | Database integrity             |
| claude/ux-polisher          | Frontend polish after real data (loading, errors, toasts)     | User experience                |
| claude/recurrence-engineer  | Correct next-instance creation, no loops                       | Advanced feature accuracy      |
| claude/uv-specialist        | uv workspace, sync, run, lockfile management                   | Build & tooling                |
| claude/feature-breaker      | Final task list for Claude Code implementation                 | Execution orchestration        |

## 5. Recommended Execution Sequence (Claude Code Prompts Order)

1. Setup uv workspace & root pyproject.toml + uv sync
2. Create missing folders (alembic/, tests/) & basic files
3. Fix backend auth.py, db.py for root .env loading
4. Remove all frontend mock data & update api.ts
5. Run type/lint checks → fix every import/syntax/comma error
6. Test full E2E flow: register → create recurring task → complete → new task

## RESPONSE_TEXT
Created comprehensive implementation plan for full-stack todo app integration including research, data model, API contracts, and quickstart guide.

## OUTCOME_IMPACT
Successfully created a complete implementation plan for the full-stack todo app integration, covering all aspects from research and architecture to deployment. The plan includes detailed documentation of data models, API contracts, and setup instructions to ensure successful integration of frontend and backend systems.

## TESTS_SUMMARY
none

## FILES_SUMMARY
- specs/1-full-stack-integration/plan.md
- specs/1-full-stack-integration/research.md
- specs/1-full-stack-integration/data-model.md
- specs/1-full-stack-integration/contracts/openapi-contract.yaml
- specs/1-full-stack-integration/quickstart.md
- specs/1-full-stack-integration/agent-context-update.md

## NEXT_PROMPTS
/sp.tasks - Generate detailed tasks for implementation

## REFLECTION_NOTE
The implementation plan comprehensively covers the integration of frontend and backend systems, with attention to security, performance, and maintainability.

## Failure modes observed:
None

## Next experiment to improve prompt quality:
None needed