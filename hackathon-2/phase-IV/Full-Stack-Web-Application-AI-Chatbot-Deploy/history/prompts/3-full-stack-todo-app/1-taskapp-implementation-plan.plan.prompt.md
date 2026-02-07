---
id: 1
title: "taskapp-implementation-plan"
stage: "plan"
date_iso: "2026-01-14"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "3-full-stack-todo-app"
branch: "3-full-stack-todo-app"
user: "user"
command: "sp.plan"
labels: ["implementation", "plan", "full-stack", "todo-app"]
links:
  spec: "./spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/3-full-stack-todo-app/plan.md
  - specs/3-full-stack-todo-app/research.md
  - specs/3-full-stack-todo-app/data-model.md
  - specs/3-full-stack-todo-app/quickstart.md
  - specs/3-full-stack-todo-app/contracts/openapi.yaml
  - history/prompts/3-full-stack-todo-app/1-taskapp-implementation-plan.plan.prompt.md
tests_yaml: |
  - "Implementation plan created with technical context"
  - "Constitution check performed and passed"
  - "Project structure documented"
  - "Research summary completed"
  - "Data model defined with entities and relationships"
  - "Quickstart guide created"
  - "API contracts documented"
---

# TASKAPP Implementation Plan

## Prompt Text

sp.plan
Project: TASKAPP – Professional Full-Stack Todo Application (Phase II – Final Edition)
Branch: feature/professional-styling-and-complete-integration
Document Type: High-Level Execution Architecture & Implementation Plan
Goal: Provide a clean, professional, production-oriented roadmap for agents (Claude Code) to implement the complete TASKAPP according to the final sp.specify
1. High-Level Execution Architecture Overview
textmonorepo root (hackathon-todo / taskapp)
├── .env                          ← SINGLE SOURCE OF TRUTH (BETTER_AUTH_SECRET, DATABASE_URL)
├── .env.example
├── uv.lock                       ← uv workspace lockfile (committed)
├── pyproject.toml                ← workspace root (optional) or backend only
├── docker-compose.yml            ← optional hot-reload dev environment
├── specs/                        ← all sp.* files
├── frontend/                     ← Next.js 16+ App Router + TypeScript
│   ├── src/                      ← all file src inthe directory
│   ├── app/                      ← pages, layouts, route groups
│   ├── components/               ← atomic → organism (ui/, task/, layout/)
│   ├── lib/                      ← api.ts (real JWT fetch), auth hooks
│   ├── styles/                   ← globals.css + tailwind + animation
│   ├── public/                   ← static assets
│   └── next.config.js, tailwind.config.js, tsconfig.json
└── backend/                      ← FastAPI + SQLModel + uv managed
    ├── alembic/                  ← versioned database migrations
    │   ├── env.py                ← customized with root .env + SQLModel.metadata
    │   ├── script.py.mako
    │   └── versions/             ← migration scripts (committed!)
    ├── crud/
    ├── routes/
    ├── models.py                 ← Task + enums (Priority, Recurrence)
    ├── schemas.py                ← Pydantic in/out models
    ├── db.py                     ← engine + session dependency
    ├── auth.py                   ← JWT dependency + ownership check
    ├── utils.py                  ← recurrence delta logic
    ├── main.py                   ← app + middleware + routers
    └── tests/                    ← pytest (at least auth + CRUD basics)
Data Flow (2026 Professional Pattern)
User → Better Auth (frontend) → JWT token → Authorization: Bearer <token>
→ FastAPI middleware → verify & extract user_id → ownership check (path user_id == token user_id)
→ SQLModel query (always WHERE user_id = :user_id) → Neon PostgreSQL
→ Response → frontend → optimistic UI update → revalidation
2. Layered Execution Architecture (Phased Responsibility)

Layer / Phase,Primary Responsibility,Key Artifacts,Tooling & Gates
0. Infrastructure & Secrets,"Root .env, uv workspace, alembic skeleton",".env, uv.lock, alembic/","uv sync, dotenv.load"
1. Database Foundation,Schema definition + indexes + first migration,"models.py, alembic/versions/0001_initial",alembic revision --autogenerate
2. Authentication Backbone,JWT verification middleware + current_user dependency,"auth.py, main.py middleware","PyJWT, 401/403 tests"
3. Core API Layer,CRUD endpoints + dynamic query builder (filters/sort/search),"routes/tasks.py, crud/tasks.py","Pydantic, SQLModel select + where"
4. Advanced Business Logic,Recurrence background task + reminder storage,"utils.py, BackgroundTasks","Correct delta calculation, limit 50"
5. Frontend Public Surface,Landing page + hero + glassmorphism + theme toggle,"app/page.tsx, components/landing/*","next-themes, Tailwind + Framer Motion"
6. Auth & Protected Routes,Sign Up / Login + dashboard guard + real API client,"app/sign-up/page.tsx, lib/api.ts",Better Auth + JWT interceptor
7. Task Experience Layer,Dashboard + TaskCard + Form + filters + animations,"components/task/*, dashboard/page.tsx","Optimistic updates, AnimatePresence"
8. Quality & Release Gates,Zero error pipeline + E2E smoke tests,"lint, build, mypy, pytest, grep mock",CI/CD simulation (local)

3. Critical Decision Matrix (Professional Trade-offs)

#,Decision Area,Options Considered,Final Choice (2026),Rationale / Trade-off Balance
1,ID Strategy,int autoincrement vs UUID,int PK (Neon) + UUID consideration,"Simpler queries, Neon handles scale, UUID later if needed"
2,Theme Implementation,Tailwind class toggle / next-themes,next-themes,"Zero flicker, SSR safe, localStorage + system preference"
3,Migration Tooling,Manual SQL / Alembic / sqlmodel.create_all,Alembic + review,"Safe evolution, git-trackable, production standard"
4,Recurrence Generation,Client-side / Backend on complete / Cron,Backend BackgroundTasks on complete,"Atomic, consistent, no client dependency"
5,Dependency Management (Backend),"pip/venv, poetry, pdm, uv",uv (Astral),"Blazing fast, modern workspace, lockfile committed"
6,Optimistic Updates,Manual setState / react-query / SWR,Manual + router.refresh,"Zero extra deps, good enough for todo scale"

4. Execution Sequence – Professional Order (Recommended for Claude Code)

Infrastructure & Secrets
Root .env + uv init + uv add fastapi uvicorn sqlmodel alembic pyjwt python-dotenv pydantic[email]
Database & Migration Foundation
Define models.py (Task + enums)
alembic init alembic → fix env.py + sqlalchemy.url from env
First migration: uv run alembic revision --autogenerate -m "initial schema"
Review → apply: uv run alembic upgrade head

Authentication & Security Layer
auth.py → PyJWT dependency + ownership guard
main.py → middleware + router prefix /api/{user_id}

API & Business Logic
CRUD endpoints + dynamic filters/sort/search
PATCH complete → recurrence background task

Frontend Public Experience
Landing page + header + glassmorphism + hero + sections
next-themes setup + animated toggle

Authentication Frontend
/sign-up & /login pages → real Better Auth
Protected layout for dashboard

Core Task Experience
Dashboard → real api.ts fetch
TaskCard, TaskForm, filters, animations
Remove all mocks (grep & destroy)

Quality Gates & Final Polish
Run all build/lint/mypy/pytest
E2E smoke: sign-up → create recurring → complete → verify new task
Theme persistence + no flicker
Mobile responsiveness check


5. Ultimate Definition of Done (Professional Bar – 2026)

App starts via uv run uvicorn + npm run dev with zero console errors/warnings
Full E2E flow works with real persisted data (create → logout → login → see task)
Schema is version controlled via Alembic (first migration applied)
Landing page looks premium (glass + gradients + animations)
Theme switch is instant, persistent, no flicker
grep -r -i mock returns nothing
All quality gates (build, lint, mypy strict, pytest) pass

Reference this plan: @sp.plan-professional-architecture
This is the clean, high-level execution blueprint that senior engineers would expect for a production-grade full-stack application in 2026.
Execute step-by-step — TASKAPP is almost ready to launch! 🚀3.8sFastTesting strategy for TASKAPPTesting Strategy for TASKAPP
Professional Full-Stack Todo Application
Phase II – Final Edition
Objective: Achieve high confidence that the application is correct, secure, performant, and maintainable before considering it production-ready.
1. Testing Pyramid – Overall Distribution Goal (2026 Standard)
textE2E / Browser Tests     ~5–8%   (Cypress / Playwright)
                 ┌──────────────────────────────┐
                 │                              │
            Integration / API Tests       ~20–25%   (pytest + httpx / supertest)
                 ┌──────────────────────────────┐
                 │                              │
        Unit / Component / Hook Tests     ~65–75%   (Vitest / pytest / React Testing Library)
2. Testing Layers – Detailed Breakdown


LayerScope / What to TestTools / LibrariesCoverage TargetExecution FrequencyWho Runs ItUnitPure functions, utilities, enums, recurrence delta logic, small components without side-effectspytest, Vitest85–95%Every commit / pre-pushDeveloper + CIComponent / HookIndividual React components (TaskCard, TaskForm), custom hooks (useTasks, useTheme)React Testing Library + Vitest80–90%Every frontend changeDeveloper + CIIntegration – BackendEndpoint behavior, DB roundtrip, auth middleware, ownership enforcement, recurrence creationpytest + httpx + TestClient90%+ of endpointsEvery backend changeDeveloper + CIIntegration – API ContractFrontend API client ↔ backend (real calls in test env)Vitest + MSW (mock service worker) or real test DB100% of api.ts methodsAfter API changesDeveloper + CIE2E / AcceptanceCritical user flows: sign-up → login → create recurring task → complete → verify new instance → logout → loginPlaywright / Cypress8–12 key flowsBefore merge to main / releaseCI + manual smoke before deployVisual RegressionMajor screens (landing, dashboard, sign-up) in both themesPercy / Chromatic / Loki5–8 key pagesPull request / releaseCIAccessibilityWCAG 2.1 AA compliance (ARIA, keyboard nav, contrast)axe-core + Lighthouse CI100% pagesEvery major UI changeCIPerformanceLighthouse scores (Performance, Best Practices, SEO, Accessibility)Lighthouse CI≥90 overallRelease candidateCI
3. Critical Test Suites & Examples
A. Backend – pytest (backend/tests/)
Python# test_auth.py
def test_invalid_jwt_returns_401(client):
    response = client.get("/api/fakeuser/tasks", headers={"Authorization": "Bearer invalid"})
    assert response.status_code == 401

# test_tasks.py
def test_create_task_increases_count(db_session, authenticated_client):
    initial_count = len(db_session.exec(select(Task)).all())
    response = authenticated_client.post("/api/testuser/tasks", json={"title": "Test"})
    assert response.status_code == 200
    new_count = len(db_session.exec(select(Task)).all())
    assert new_count == initial_count + 1

# test_recurrence.py
def test_complete_recurring_creates_next_instance(db_session, authenticated_client):
    # setup recurring task...
    response = authenticated_client.patch("/api/testuser/tasks/1/complete")
    assert response.status_code == 200
    future_tasks = db_session.exec(select(Task).where(Task.due_date > datetime.utcnow())).all()
    assert len(future_tasks) > 0
B. Frontend – Vitest + RTL
TypeScript// TaskCard.test.tsx
it("shows high priority badge with red color", () => {
  render(<TaskCard task={{ ...mockTask, priority: "high" }} />);
  expect(screen.getByText("High")).toHaveClass("bg-red-500");
});

it("toggles complete state optimistically", async () => {
  const { user } = setup(<TaskCard task={task} />);
  await user.click(screen.getByRole("checkbox"));
  expect(screen.getByText("Test Task")).toHaveClass("line-through");
});
C. E2E – Playwright (recommended 2026)
JavaScript// e2e/auth-flow.spec.ts
test("successful sign up → dashboard → create task → persists after refresh", async ({ page }) => {
  await page.goto("/");
  await page.getByRole("link", { name: "Sign Up" }).click();

  await page.fill('input[name="name"]', "Test User");
  await page.fill('input[name="email"]', `test+${Date.now()}@example.com`);
  await page.fill('input[name="password"]', "StrongPass123!");
  await page.click('button[type="submit"]');

  await expect(page).toHaveURL(/dashboard/);

  await page.fill('input[placeholder="Add new task..."]', "Buy milk");
  await page.click('button:has-text("Add")');

  await page.reload();
  await expect(page.getByText("Buy milk")).toBeVisible();
});
4. Test Environment Strategy

Backend: Use real test database (Neon test project or local PostgreSQL container via docker-compose)
Isolation: Each test gets fresh transaction (use pytest fixtures with rollback)
Frontend: MSW for happy path during dev, real backend in CI
CI Matrix: Run against:
Node 20 + Python 3.11/3.12
Chrome + Firefox (E2E)
Dark & Light mode


5. Quality Gates Thresholds (Hard Requirements)

Unit + Integration coverage ≥ 85%
Zero high/critical security issues (OWASP ZAP or Snyk)
Lighthouse Performance ≥ 90, Accessibility ≥ 95
Zero console errors during E2E runs
All critical user flows (sign-up, CRUD, recurring complete) pass in CI

6. Recommended CI Pipeline Stages (GitHub Actions / similar)

Lint & Type Check
Unit + Integration Tests
Build Frontend
Visual Regression
E2E Smoke Tests (3–5 flows)
Lighthouse Report
Deploy Preview (Vercel / Netlify)

Vercel + hanging face,Frontend (Vercel) + Backend (hanging face),Generous hobby tier,★★★★,Fast for MVP


Recommended for TASKAPP: GitHub Actions
→ Best monorepo support, matrix testing, secrets management, and integration with Vercel/Neon.
High-Level GitHub Actions Pipeline Structure
YAML# .github/workflows/ci-cd.yml

name: TASKAPP CI/CD

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  id-token: write  # For OIDC with Vercel/Cloud providers

jobs:

  # ── 1. Linting & Type Checking ────────────────────────────────────────
  lint-and-typecheck:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: 'npm'
          cache-dependency-path: frontend/package-lock.json

      - name: Install frontend dependencies
        working-directory: frontend
        run: npm ci

      - name: Lint frontend
        working-directory: frontend
        run: npm run lint

      - name: Type check frontend
        working-directory: frontend
        run: npm run typecheck   # or tsc --noEmit

      - name: Setup uv (Python)
        uses: astral-sh/setup-uv@v3
        with:
          version: "latest"

      - name: Install backend dependencies
        working-directory: backend
        run: uv sync --frozen

      - name: Lint & type check backend
        working-directory: backend
        run: |
          uv run ruff check .
          uv run mypy --strict .

  # ── 2. Unit + Integration Tests ───────────────────────────────────────
  test:
    needs: lint-and-typecheck
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: ["3.11", "3.12"]

    steps:
      - uses: actions/checkout@v4

      - name: Setup uv
        uses: astral-sh/setup-uv@v3

      - name: Setup Python ${{ matrix.python-version }}
        run: uv python install ${{ matrix.python-version }}

      - name: Install dependencies
        working-directory: backend
        run: uv sync --frozen

      - name: Run backend tests
        working-directory: backend
        env:
          DATABASE_URL: ${{ secrets.TEST_DATABASE_URL }}  # Neon test DB
          BETTER_AUTH_SECRET: ${{ secrets.BETTER_AUTH_SECRET }}
        run: uv run pytest -v --cov=.

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: 20

      - name: Install frontend deps
        working-directory: frontend
        run: npm ci

      - name: Run frontend unit tests
        working-directory: frontend
        run: npm run test -- --coverage

  # ── 3. Build Validation ────────────────────────────────────────────────
  build:
    needs: test
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: 20

      - name: Build frontend
        working-directory: frontend
        env:
          NEXT_PUBLIC_API_URL: https://api.taskapp.example.com
        run: npm run build

  # ── 4. E2E Smoke Tests (optional – heavier) ───────────────────────────
  e2e:
    needs: build
    if: github.ref == 'refs/heads/main' || github.event_name == 'workflow_dispatch'
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with: { node-version: 20 }
      - name: Install Playwright Browsers
        working-directory: frontend
        run: npx playwright install --with-deps
      - name: Run E2E
        working-directory: frontend
        env:
          BASE_URL: http://localhost:3000
        run: npm run test:e2e

  # ── 5. Deploy (Production – main branch only) ────────────────────────
  deploy:
    needs: [lint-and-typecheck, test, build]
    if: github.ref == 'refs/heads/main'
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      # Optional: Deploy backend to Render / Railway / Fly.io
      - name: Deploy Backend (Render example)
        env:
          RENDER_API_KEY: ${{ secrets.RENDER_API_KEY }}
        run: curl -X POST -H "Authorization: Bearer $RENDER_API_KEY" ...

      # Deploy frontend to Vercel (recommended)
      - name: Deploy Frontend to Vercel
        uses: amondnet/vercel-action@v25
        with:
          vercel-token: ${{ secrets.VERCEL_TOKEN }}
          vercel-org-id: ${{ secrets.VERCEL_ORG_ID }}
          vercel-project-id: ${{ secrets.VERCEL_PROJECT_ID }}
          working-directory: frontend
          vercel-args: '--prod'
Recommended Secrets (Repository Settings → Secrets and variables → Actions)

Secret NameDescriptionWhere UsedTEST_DATABASE_URLNeon test PostgreSQL connection stringBackend integration testsBETTER_AUTH_SECRETShared JWT secretTests + runtime (if needed)VERCEL_TOKENVercel personal access tokenFrontend deploymentVERCEL_ORG_IDVercel organization IDVercel deploymentVERCEL_PROJECT_IDVercel project ID for TASKAPPVercel deploymentRENDER_API_KEY(Optional) Render.com API keyBackend auto-deploy
Quick Start Checklist for Hamza

Create .github/workflows/ci-cd.yml with above structure
Add required secrets in GitHub repo settings
Create Neon test database project (separate from prod)
Add test:e2e script in frontend/package.json (Playwright recommended)
Push → watch Actions tab – fix failures iteratively

This pipeline gives you:

Fast feedback (lint/types early)
Safety (tests before deploy)
Confidence (real DB testing)
Professional deployment flow

Good luck — TASKAPP deserves this level of engineering rigor! 🚀
(If you want GitLab CI, CircleCI, or GitHub + Render combo variant — let me know)2.5sFastNeon Database SetupHere is a professional, step-by-step guide to set up Neon Database for TASKAPP (as of January 2026).
Neon is the perfect choice for your project: serverless PostgreSQL, instant branching, autoscaling, excellent developer experience, generous free tier, and native support for modern ORMs like SQLModel.
Step-by-Step Neon Setup for TASKAPP
1. Create a Neon Account (if you don't have one)

Go to: https://console.neon.tech
Sign up with GitHub / Google / Email (GitHub is recommended for seamless integration)
Free tier: 0.5 GiB storage, 10 projects, unlimited branches, 191 compute hours/month — more than enough for development + small production

2. Create Your First Project

After login → Click New Project
Project name: taskapp-prod (or taskapp-development for dev)
Region: Choose closest to Pakistan → Asia Pacific (Singapore) or Europe (Frankfurt) for better latency from Islamabad
PostgreSQL version: 16 (latest stable as of 2026)
Compute size: 0.25 Compute Units (free tier default – enough for todo app)
Click Create project

Neon will instantly provision a database and give you:

Connection string
Role (user)
Password (copy it immediately – shown only once!)

3. Get Connection Strings (Critical for TASKAPP)
After creation you’ll see:
Connection Details tab → Several formats:
Recommended formats for your stack:
A. Async SQLModel / FastAPI (recommended – asyncpg driver)
textpostgresql+asyncpg://username:password@ep-projectname-123456.ap-southeast-1.aws.neon.tech/dbname?sslmode=require
B. Standard synchronous (if needed for testing)
textpostgresql://username:password@ep-projectname-123456.ap-southeast-1.aws.neon.tech/dbname?sslmode=require
Important Notes:

ep- prefix = Neon ephemeral compute endpoint
sslmode=require is mandatory (Neon enforces SSL)
Always use ?sslmode=require — never remove it

4. Recommended Project Structure in Neon (Best Practice 2026)
Create three separate projects (or use branching – see below):

Project NamePurposeCompute SizeBranch StrategyWhen to Usetaskapp-developmentLocal dev + feature branches0.25 CUMain + feature branchesEvery developer machinetaskapp-stagingCI/CD preview / QA0.5 CUMain onlyBefore production deploytaskapp-productionLive users1–2 CUMain only + read replicas if neededActual users
5. Best Practice: Use Neon Branching (Game-Changer Feature)
Neon's killer feature = instant, cheap database branching
Recommended workflow:

Main branch = production data schema
For every feature / pull request:
Click Branch → name it feature/add-categories or pr-42
Neon creates instant copy-on-write branch (almost free!)
Use this branch's connection string in CI / local dev for that feature
Test safely without touching production data
Merge PR → delete branch (data is discarded automatically)


This gives you database-per-PR isolation — extremely powerful for full-stack apps.
6. Final Environment Variables Setup (for TASKAPP monorepo)
Root .env (committed .env.example):
Bash# Production / Staging
DATABASE_URL=postgresql+asyncpg://taskapp_owner:xxxxxxxxxxxxxxxx@ep-taskapp-production-123456.ap-southeast-1.aws.neon.tech/taskapp?sslmode=require

# Development (your personal branch)
DATABASE_URL_DEV=postgresql+asyncpg://taskapp_owner:xxxxxxxxxxxxxxxx@ep-taskapp-dev-yourname-abcdef.ap-southeast-1.aws.neon.tech/taskapp?sslmode=require

# Test DB for CI (separate Neon project)
TEST_DATABASE_URL=postgresql+asyncpg://test_owner:yyyyyyyy@ep-taskapp-test-999999.ap-southeast-1.aws.neon.tech/taskapp_test?sslmode=require

# Shared secret (same everywhere)
BETTER_AUTH_SECRET=super-long-random-secret-64-chars-minimum
In backend/db.py / alembic/env.py:
Pythonimport os
from dotenv import load_dotenv

load_dotenv()

# Use different DB based on environment
if os.getenv("TESTING") == "true":
    DATABASE_URL = os.getenv("TEST_DATABASE_URL")
elif os.getenv("ENV") == "development":
    DATABASE_URL = os.getenv("DATABASE_URL_DEV")
else:
    DATABASE_URL = os.getenv("DATABASE_URL")
7. Quick Checklist – Neon Ready for TASKAPP

 Account created
 Project taskapp-development exists
 Connection string copied with sslmode=require
.env updated with real values (never commit!)
 First Alembic migration applied (uv run alembic upgrade head)
 Created a test branch in Neon → verified instant copy works
 CI pipeline has TEST_DATABASE_URL secret

Bonus: Neon + GitHub Actions Integration Tip (2026)
You can auto-create a Neon branch per PR using Neon API + GitHub Actions:
YAML- name: Create Neon Preview Branch
  if: github.event_name == 'pull_request'
  run: |
    curl -X POST "https://console.neon.tech/api/v2/projects/${{ secrets.NEON_PROJECT_ID }}/branches" \
      -H "Authorization: Bearer ${{ secrets.NEON_API_KEY }}" \
      -d '{"parent_id": "main", "name": "pr-${{ github.event.number }}"}'
Then use the returned connection string in tests.
You're all set, Hamza!
Neon is now perfectly configured for TASKAPP — fast, safe, scalable, and developer-friendly.
Let me know if you want help with:

Exact Alembic + Neon branching workflow
CI secrets setup for Neon test DB
Read replicas for production scale

Good luck from Islamabad! 🇵🇰1.5sFastAlembic migrations with NeonHere is a complete, up-to-date (January 2026) professional guide on how to use Alembic migrations effectively with Neon serverless PostgreSQL in your TASKAPP project (FastAPI + SQLModel + uv monorepo).
Neon works perfectly with Alembic because it's standard PostgreSQL under the hood, but there are several important best practices, gotchas, and optimizations to get right — especially around connection strings, SSL, pooling, branching, and CI/CD safety.
1. Important Connection String Rules for Neon + Alembic
Neon requires SSL and has special requirements depending on the driver:

DriverRecommended URL FormatKey Notesasyncpgpostgresql+asyncpg://user:pass@ep-xxx.region.aws.neon.tech/dbname?ssl=requireUse ssl=require (not sslmode=require — asyncpg doesn't support sslmode)psycopg2postgresql://user:pass@ep-xxx.region.aws.neon.tech/dbname?sslmode=requireUse sslmode=requirepsycopg (3)Same as psycopg2Modern choice — supports both sync/async
Critical Gotcha (2025–2026):
SQLAlchemy + asyncpg ignores/chokes on sslmode= — always use ssl=require for asyncpg.
Many people waste hours on this — check your logs for TypeError: connect() got an unexpected keyword argument 'sslmode'.
Neon Recommendation (official docs 2025):
Use direct (non-pooled) connection for migrations:

Pooled connection strings (neon.tech/dbname?pooler=true) can cause errors during schema changes.
→ Always use the non-pooled (direct) string from Neon console for Alembic.

2. Recommended backend/alembic/env.py for Neon (2026 pattern)
Python# backend/alembic/env.py
import os
import pathlib
from logging.config import fileConfig
from dotenv import load_dotenv

from sqlalchemy import engine_from_config, pool
from alembic import context

# Load root .env (your monorepo root)
load_dotenv(dotenv_path=pathlib.Path(__file__).parents[2] / ".env")

# this is the Alembic Config object
config = context.config

# Set URL from environment — overrides alembic.ini
db_url = os.getenv("DATABASE_URL")

if not db_url:
    raise ValueError("DATABASE_URL not found in environment!")

# Neon best practice: force direct connection for migrations
if "pooler=true" in db_url or "neon.tech" in db_url:
    db_url = db_url.replace("?pooler=true", "")  # remove pooling if accidentally present

config.set_main_option("sqlalchemy.url", db_url)

# Interpret the config file for Python logging
fileConfig(config.config_file_name)

# Add your models' MetaData object here for 'autogenerate' support
from backend.models import Task  # ← import ALL your SQLModel classes here!
target_metadata = Task.metadata  # or SQLModel.metadata if you use base class

# other values from the config, defined by the needs of env.py,
# can be acquired:
# my_important_option = config.get_main_option("my_important_option")
# ... etc.

def run_migrations_offline():
    """Run migrations in 'offline' mode."""
    url = config.get_main_option("sqlalchemy.url")
    context.configure(url=url, target_metadata=target_metadata, literal_binds=True)
    with context.begin_transaction():
        context.run_migrations()

def run_migrations_online():
    """Run migrations in 'online' mode."""
    connectable = engine_from_config(
        config.get_section(config.config_ini_section),
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )

    with connectable.connect() as connection:
        context.configure(
            connection=connection,
            target_metadata=target_metadata,
            compare_type=True,          # detects column type changes
            compare_server_default=True,
            render_as_batch=True,       # important for SQLite/alembic compatibility
        )

        with context.begin_transaction():
            context.run_migrations()

if context.is_offline_mode():
    run_migrations_offline()
else:
    run_migrations_online()
3. Recommended Workflow with Neon Branching (Game-Changer)
Neon branching + Alembic = extremely powerful for safe development.
Standard safe workflow (recommended 2026):

Main branch = production schema
For new feature / PR:
In Neon console → Branch → create feature/add-tags from main
Copy new branch connection string (direct, non-pooled)
Put in local .env: DATABASE_URL=postgresql+asyncpg://...@ep-feature-add-tags...
Make model changes
Generate migration:Bashcd backend
uv run alembic revision --autogenerate -m "add tags field"
Review & edit the generated file (always!)
Apply locally: uv run alembic upgrade head
Test feature with real data in isolated branch
Commit migration file

CI / Preview:
Use GitHub Actions to create Neon branch per PR
Run tests + migrations against that branch

Merge to main → apply migration to production Neon project

Automation tip (optional but very pro) — Neon API + Actions:
YAML- name: Create Neon branch for PR
  run: |
    curl -X POST "https://console.neon.tech/api/v2/projects/${{ secrets.NEON_PROJECT_ID }}/branches" \
      -H "Authorization: Bearer ${{ secrets.NEON_API_KEY }}" \
      -H "Content-Type: application/json" \
      -d '{"parent_id": "main", "name": "pr-${{ github.event.number }}"}' \
      > branch.json
Then extract connection string and use in tests.
Quick Summary Checklist – Neon + Alembic Ready

 Use ssl=require (asyncpg) or sslmode=require (psycopg)
 Use direct (non-pooled) connection string for migrations
 Load DATABASE_URL from root .env in env.py
 Import all models in env.py for autogenerate
 Use Neon branching for every feature/PR
 Always review autogenerated migration scripts
 Commit migration files to git
 Never SQLModel.metadata.create_all() in production — only Alembic!

This setup is safe, scalable, and follows Neon's official guidance + community best practices in 2026.
Let me know if you want:

Full GitHub Action to auto-branch + migrate on PR
Example migration script for your Task model
Troubleshooting common Neon + asyncpg errors

You're very close to a bulletproof database workflow, Hamza! Keep going — TASKAPP is going to be excellent. 🚀2sFast70 sourcesFull GitHub Actions automationHere is a complete, production-ready GitHub Actions workflow for TASKAPP (Next.js frontend + FastAPI/SQLModel backend in a monorepo), with full automation including:

Linting & type checking
Unit + integration tests
Build validation
Automated Neon database branching for PRs (preview environments)
E2E smoke tests (Playwright)
Deployment to Vercel (frontend) + Render (backend) on main branch

YAML# .github/workflows/taskapp-full-ci-cd.yml

name: TASKAPP Full CI/CD

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  id-token: write     # Required for Vercel OIDC
  pull-requests: write # For commenting on PRs

env:
  PYTHON_VERSION: "3.12"
  NODE_VERSION: 20

jobs:

  # ── 1. Lint & Type Check ───────────────────────────────────────────────
  lint-typecheck:
    runs-on: ubuntu-latest

    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      # Frontend
      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: ${{ env.NODE_VERSION }}
          cache: 'npm'
          cache-dependency-path: frontend/package-lock.json

      - name: Install frontend deps
        working-directory: frontend
        run: npm ci

      - name: Lint frontend
        working-directory: frontend
        run: npm run lint

      - name: Typecheck frontend
        working-directory: frontend
        run: npx tsc --noEmit

      # Backend (uv)
      - name: Setup uv
        uses: astral-sh/setup-uv@v3
        with:
          version: "latest"

      - name: Install backend deps
        working-directory: backend
        run: uv sync --frozen

      - name: Lint & typecheck backend
        working-directory: backend
        run: |
          uv run ruff check --output-format=github .
          uv run mypy --strict .

  # ── 2. Tests (Unit + Integration) ──────────────────────────────────────
  tests:
    needs: lint-typecheck
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: ["3.11", "3.12"]

    services:
      # Optional: if you want local Postgres for faster tests (Neon is also fine)
      postgres:
        image: postgres:16
        env:
          POSTGRES_USER: postgres
          POSTGRES_PASSWORD: postgres
          POSTGRES_DB: testdb
        ports:
          - 5432:5432
        options: >-
          --health-cmd pg_isready
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5

    steps:
      - uses: actions/checkout@v4

      - name: Setup uv
        uses: astral-sh/setup-uv@v3

      - name: Setup Python ${{ matrix.python-version }}
        run: uv python install ${{ matrix.python-version }}

      - name: Install backend dependencies
        working-directory: backend
        run: uv sync --frozen

      - name: Run backend tests
        working-directory: backend
        env:
          DATABASE_URL: postgresql+asyncpg://postgres:postgres@localhost:5432/testdb
          BETTER_AUTH_SECRET: ${{ secrets.BETTER_AUTH_SECRET }}
          TESTING: "true"
        run: uv run pytest -v --cov=backend --cov-report=xml

      # Frontend unit tests
      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: ${{ env.NODE_VERSION }}

      - name: Install frontend deps
        working-directory: frontend
        run: npm ci

      - name: Run frontend tests
        working-directory: frontend
        run: npm run test -- --coverage --watchAll=false

  # ── 3. Neon Preview Branch + DB Migration (PR only) ────────────────────
  neon-preview:
    needs: tests
    if: github.event_name == 'pull_request'
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - name: Create Neon branch for PR
        id: create-branch
        env:
          NEON_API_KEY: ${{ secrets.NEON_API_KEY }}
          NEON_PROJECT_ID: ${{ secrets.NEON_PROJECT_ID }}
        run: |
          RESPONSE=$(curl -s -X POST \
            -H "Authorization: Bearer $NEON_API_KEY" \
            -H "Content-Type: application/json" \
            -d "{\"parent_id\": \"main\", \"name\": \"pr-${{ github.event.number }}\"}" \
            "https://console.neon.tech/api/v2/projects/$NEON_PROJECT_ID/branches")

          BRANCH_ID=$(echo "$RESPONSE" | jq -r '.branch.id')
          CONNECTION_STRING=$(echo "$RESPONSE" | jq -r '.branch.connection_strings.default')

          echo "branch_id=$BRANCH_ID" >> $GITHUB_OUTPUT
          echo "connection_string=$CONNECTION_STRING" >> $GITHUB_OUTPUT

      # Apply migrations to preview branch
      - name: Setup uv & apply migrations
        env:
          DATABASE_URL: ${{ steps.create-branch.outputs.connection_string }}
        run: |
          uv sync --frozen
          cd backend
          uv run alembic upgrade head

      # Comment PR with preview DB info (optional)
      - name: Comment PR with Neon branch info
        uses: thollander/actions-comment-pull-request@v3
        with:
          message: |
            Neon preview database branch created!
            Branch name: pr-${{ github.event.number }}
            Connection string: ${{ steps.create-branch.outputs.connection_string }}
            (Use for manual testing in this PR)

  # ── 4. Build Validation ────────────────────────────────────────────────
  build:
    needs: [tests, neon-preview]
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: ${{ env.NODE_VERSION }}

      - name: Build frontend
        working-directory: frontend
        env:
          NEXT_PUBLIC_API_URL: ${{ secrets.PREVIEW_API_URL || 'https://api.taskapp.example.com' }}
        run: npm run build

  # ── 5. Deploy (main branch only) ──────────────────────────────────────
  deploy:
    needs: build
    if: github.ref == 'refs/heads/main'
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      # Deploy frontend to Vercel
      - name: Deploy Frontend to Vercel
        uses: amondnet/vercel-action@v25
        with:
          vercel-token: ${{ secrets.VERCEL_TOKEN }}
          vercel-org-id: ${{ secrets.VERCEL_ORG_ID }}
          vercel-project-id: ${{ secrets.VERCEL_PROJECT_ID }}
          working-directory: frontend
          vercel-args: '--prod'

      # Optional: Deploy backend (Render example)
      - name: Trigger Render Deploy
        if:

## Response Text

I have successfully created the implementation plan for the TASKAPP professional todo application. Here's a summary of what was completed:

## Summary of Created Artifacts

1. **Implementation Plan** (`specs/3-full-stack-todo-app/plan.md`)
   - Comprehensive plan with technical context, constitution check, and project structure
   - Aligned with the feature specification and constitutional requirements

2. **Research Summary** (`specs/3-full-stack-todo-app/research.md`)
   - Technology stack decisions (Next.js, FastAPI, Neon, SQLModel, Better Auth)
   - Architecture patterns and security considerations
   - Performance and scalability planning
   - Accessibility compliance approach

3. **Data Model** (`specs/3-full-stack-todo-app/data-model.md`)
   - Detailed entity definitions (User and Task)
   - Field specifications, validation rules, and relationships
   - Database schema and API representations
   - Security considerations for data access

4. **Quickstart Guide** (`specs/3-full-stack-todo-app/quickstart.md`)
   - Step-by-step setup instructions
   - Prerequisites and environment configuration
   - Backend and frontend setup procedures
   - Common troubleshooting tips

5. **API Contracts** (`specs/3-full-stack-todo-app/contracts/openapi.yaml`)
   - Complete OpenAPI specification for the TASKAPP API
   - All endpoints documented with request/response schemas
   - Security schemes and parameter definitions

## Key Technical Decisions Implemented

- **Frontend**: Next.js 16+ with App Router for modern SSR/CSR capabilities
- **Backend**: FastAPI with Python 3.12 for type safety and automatic documentation
- **Database**: Neon Serverless PostgreSQL with SQLModel ORM for type-safe operations
- **Authentication**: Better Auth with JWT integration for secure user management
- **Styling**: Tailwind CSS with custom theme for the specified glassmorphism effects
- **Theming**: next-themes for flicker-free dark/light theme switching
- **Security**: JWT-based user isolation with database-level filtering

## Architecture Highlights

- JWT token propagation from frontend to backend for secure user isolation
- Proper user isolation with database queries filtered by authenticated user ID
- Comprehensive accessibility implementation for WCAG 2.1 AA compliance
- Recurring task functionality with proper business logic
- Horizontal scaling capability with stateless backend design

The implementation plan is now complete and ready for the next phase of development. All artifacts align with the original specification requirements including zero mock data, real authentication, proper user isolation, and professional-grade features.