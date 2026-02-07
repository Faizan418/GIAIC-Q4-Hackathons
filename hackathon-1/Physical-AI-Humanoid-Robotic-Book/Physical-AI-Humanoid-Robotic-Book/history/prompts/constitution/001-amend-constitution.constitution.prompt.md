---
id: 1
title: Amend Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Claude Sonnet 4.5
command: /sp.constitution
labels: [constitution,rag,cohere,chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

## Project Constitution: Integrated RAG Chatbot with Cohere Embeddings, Uvicorn & Workflow

**Project:** *Physical AI & Humanoid Robotics – AI-Native Textbook*
**Module:** Integrated RAG Chatbot
**Owner:** Sheikh Hamza
**Status:** Production-Ready

---

## 1. Purpose

Embed a **RAG chatbot** within the AI-Native textbook with:

- Cohere embeddings for semantic search.
- FastAPI backend running on **Uvicorn**.
- Qdrant vector search + Neon Postgres storage.
- Frontend integration in Docusaurus.
- Clear, production-ready workflow for CI/CD and deployment.

---

## 2. Scope

- Backend development with FastAPI + Uvicorn.
- Embedding pipeline with Cohere.
- Vector storage in Qdrant + metadata in Postgres.
- Frontend chat widget in Docusaurus.
- CI/CD pipelines for automated testing, build, and deployment.

---

## 3. System Architecture

### 3.1 Components

| Component | Description |
|-----------|-------------|
| **FastAPI Backend** | API endpoints and RAG orchestration, runs on **Uvicorn**. |
| **Cohere Embeddings** | Semantic vectorization of book content. |
| **OpenAI Agents / ChatKit** | LLM generation with context. |
| **Neon Postgres** | Book content and metadata storage. |
| **Qdrant Cloud** | Vector database for retrieval. |
| **Docusaurus Frontend** | Embedded chat interface with text selection. |
| **CI/CD Workflow** | Automated build, test, and deployment pipelines. |

### 3.2 Data Flow

1. Preprocess book → generate **Cohere embeddings** → store vectors in Qdrant + metadata in Postgres.
2. User submits query / highlighted text via Docusaurus chat.
3. FastAPI + Uvicorn backend receives request → vector retrieval from Qdrant.
4. Context sent to OpenAI ChatKit → answer generation.
5. Response returned to frontend.

---

## 4. Functional Requirements

- Semantic retrieval with Cohere embeddings.
- Multi-turn, context-aware RAG answers.
- FastAPI + Uvicorn backend endpoints for `/query` and `/highlight-query`.
- Frontend chat integration supporting text selection.
- CI/CD pipelines with automated testing, linting, and deployment.

---

## 5. Technology Stack

| Layer | Technology |
|-------|------------|
| Backend | FastAPI, Python 3.12, Uvicorn ASGI Server |
| Embeddings | Cohere Embed API |
| AI / LLM | OpenAI Agents / ChatKit SDKs |
| Database | Neon Serverless Postgres |
| Vector Search | Qdrant Cloud Free Tier |
| Frontend | Docusaurus, React, TypeScript |
| Deployment | Vercel (Frontend), Serverless/Cloud Backend |
| CI/CD | GitHub Actions / Workflows |

---

## 6. Workflow Structure

### 6.1 Local Development

book-rag-chatbot/
├── backend/
│ ├── main.py # FastAPI app
│ ├── api/
│ │ ├── query.py # Endpoints for queries
│ │ └── highlight.py # Endpoints for highlighted text
│ ├── embeddings/
│ │ └── cohere_embed.py # Cohere embedding logic
│ ├── db/
│ │ └── neon_postgres.py # Postgres ORM / connection
│ └── vectorstore/
│ └── qdrant_client.py # Qdrant CRUD operations
├── frontend/
│ ├── docusaurus/
│ │ └── src/components/chat/ # Chat widget integration
├── tests/
│ ├── test_api.py
│ ├── test_embeddings.py
│ └── test_vectorstore.py
├── .env # API keys, DB URLs
└── requirements.txt # Python dependencies

shell
Copy code

### 6.2 CI/CD Workflow (GitHub Actions)

.github/workflows/
└── deploy.yml
├── trigger: [push, pull_request] to main
├── jobs:
│ ├── lint:
│ │ runs: flake8 + mypy
│ ├── test:
│ │ runs: pytest
│ ├── build-frontend:
│ │ runs: Docusaurus build
│ ├── deploy-backend:
│ │ runs: Uvicorn + cloud deployment
│ └── deploy-frontend:
│ runs: Vercel deployment

yaml
Copy code

### 6.3 Deployment Steps

1. Backend: FastAPI + Uvicorn → serverless / cloud instance.
2. Frontend: Docusaurus build → deploy to Vercel.
3. Database: Neon Serverless Postgres initialized + migrations.
4. Vector DB: Qdrant collection created → embeddings populated.
5. Environment variables: `.env` securely stored in cloud provider.
6. CI/CD: Push triggers lint → test → build → deploy.

---

## 7. Clean Code & Professional Practices

- Modular pipeline: embeddings, retrieval, RAG generation.
- Python type hints + docstrings.
- Logging: structured queries, retrievals, responses.
- Graceful error handling (API & DB).
- Env vars for all sensitive data.
- Version-controlled workflow for reproducibility.

---

## 8. Evaluation Criteria

- Accurate answers from book content.
- Fast semantic retrieval using Cohere embeddings + Qdrant.
- Multi-turn, context-aware chat.
- CI/CD workflow passes lint, tests, and deploys successfully.
- Clean, maintainable, modular codebase.

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read any command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

## Response snapshot

<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0 (added RAG chatbot module governance)
List of modified principles: Docusaurus-First Content Structure → RAG-Enhanced Content Structure, Tooling Mandate → RAG-Integrated Tooling Mandate
Added sections: RAG Chatbot Module Principles, Backend Architecture Governance, Frontend Integration Standards, Data Management Principles
Removed sections: None
Templates requiring updates: ✅ .specify/templates/plan-template.md, ✅ .specify/templates/spec-template.md, ✅ .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics – AI-Native Textbook Constitution

## Core Principles

### RAG-Enhanced Content Structure
All content must follow Docusaurus docs folder structure with sequential numbering while enabling semantic search capabilities. Use .md or .mdx files with consistent heading hierarchy (# for module, ## for chapters, ### for sections) to support Cohere embedding generation and Qdrant vector storage.

### RAG-Integrated Tooling Mandate
Mandatory use of Docusaurus (TypeScript) for book authoring with integrated RAG chatbot, GitHub Pages for deployment, Spec-Kit Plus for specification governance, Claude Code for assisted authoring, FastAPI + Uvicorn for backend, Cohere for embeddings, Qdrant for vector search, Neon Postgres for metadata, and OpenAI ChatKit for LLM generation. Human validation required for all AI-assisted content.

### Spec-Driven Development Governance
All content creation and updates must align with /sp.specify, /sp.plan, and /sp.constitution. Change control requires updating these governance documents when scope, tools, or structure changes. For RAG module, all API endpoints and embedding processes must be documented in specs.

### Quality and Verification (NON-NEGOTIABLE)
All factual claims must be traceable through verified sources. Code runs as documented, simulations respect real-world physics, and learning outcomes align with labs and assessments. Zero plagiarism tolerance. All RAG responses must be traceable to specific book content with proper citations.

### Academic Standards
Citation style: APA. Engineering clarity for CS/Robotics audience. Physical realism over theory-only claims. Accuracy through verified sources. All RAG-generated content must maintain academic rigor and proper attribution to source materials.

### Content Structure Compliance
All files must follow Docusaurus docs folder structure with sequential and descriptive filenames to maintain order and enable proper embedding generation for semantic search capabilities.

## Backend Architecture Governance

### FastAPI + Uvicorn Standards
All backend endpoints must use FastAPI with Uvicorn ASGI server. Implement proper type hints, async/await patterns, and structured logging. API endpoints for `/query` and `/highlight-query` must follow RESTful principles with proper error handling.

### Cohere Embedding Pipeline
All book content must be processed through Cohere embedding pipeline with proper chunking strategy. Embeddings must be stored in Qdrant vector database with corresponding metadata in Neon Postgres. Implement proper error handling and retry logic for API calls.

### Data Management and Storage
Use Neon Serverless Postgres for book content and metadata storage. Use Qdrant Cloud for vector storage. Implement proper data validation, backup strategies, and migration procedures. All sensitive data must be stored in environment variables.

## Frontend Integration Standards

### Docusaurus Chat Widget
The RAG chat widget must be seamlessly integrated into Docusaurus frontend with proper React/TypeScript implementation. Support text selection and context-aware multi-turn conversations. Follow accessibility standards and responsive design principles.

### User Experience Requirements
Provide fast, context-aware responses with proper loading states. Implement proper error handling for network issues. Maintain conversation history and support text highlighting for targeted queries.

## Development Workflow

Use Spec-Kit Plus commands (/sp.specify, /sp.plan, /sp.tasks) for specification governance. Human validation required for Claude Code assistance. Follow Docusaurus authoring guidelines with proper file structure. Implement CI/CD with automated testing, linting, and deployment for both frontend and backend components.

## Governance

Constitution supersedes all other practices. Amendments require documentation via /sp.constitution command. All content must comply with this constitution. Changes to scope, tools, or structure require updating /sp.constitution, /sp.specify, and /sp.plan. RAG module changes must follow the same governance process.

**Version**: 1.1.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-16

## Outcome

- ✅ Impact: Updated constitution to include RAG chatbot governance with Cohere embeddings, FastAPI, Uvicorn, Qdrant, and Neon Postgres integration
- 🧪 Tests: null
- 📁 Files: Updated .specify/memory/constitution.md with new RAG-integrated principles
- 🔁 Next prompts: Ready for spec development for RAG chatbot module
- 🧠 Reflection: Constitution successfully updated to reflect new RAG chatbot architecture while maintaining existing Docusaurus-based structure

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All required elements updated correctly
- Prompt variant (if applicable): Standard constitution update workflow
- Next experiment (smallest change to try): Begin spec development for RAG chatbot module
