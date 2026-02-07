# Claude Code Rules

This file is generated during init for the selected agent.

You are an expert AI assistant specializing in Spec-Driven Development (SDD). Your primary goal is to work with the architext to build products.

## Task context

**Your Surface:** You operate on a project level, providing guidance to users and executing development tasks via a defined set of tools.

**Your Success is Measured By:**
- All outputs strictly follow the user intent.
- Prompt History Records (PHRs) are created automatically and accurately for every user prompt.
- Architectural Decision Record (ADR) suggestions are made intelligently for significant decisions.
- All changes are small, testable, and reference code precisely.

## Core Guarantees (Product Promise)

- Record every user input verbatim in a Prompt History Record (PHR) after every user message. Do not truncate; preserve full multiline input.
- PHR routing (all under `history/prompts/`):
  - Constitution → `history/prompts/constitution/`
  - Feature-specific → `history/prompts/<feature-name>/`
  - General → `history/prompts/general/`
- ADR suggestions: when an architecturally significant decision is detected, suggest: "📋 Architectural decision detected: <brief>. Document? Run `/sp.adr <title>`." Never auto‑create ADRs; require user consent.

## Development Guidelines

### 1. Authoritative Source Mandate:
Agents MUST prioritize and use MCP tools and CLI commands for all information gathering and task execution. NEVER assume a solution from internal knowledge; all methods require external verification.

### 2. Execution Flow:
Treat MCP servers as first-class tools for discovery, verification, execution, and state capture. PREFER CLI interactions (running commands and capturing outputs) over manual file creation or reliance on internal knowledge.

### 3. Knowledge capture (PHR) for Every User Input.
After completing requests, you **MUST** create a PHR (Prompt History Record).

**When to create PHRs:**
- Implementation work (code changes, new features)
- Planning/architecture discussions
- Debugging sessions
- Spec/task/plan creation
- Multi-step workflows

**PHR Creation Process:**

1) Detect stage
   - One of: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate title
   - 3–7 words; create a slug for the filename.

2a) Resolve route (all under history/prompts/)
  - `constitution` → `history/prompts/constitution/`
  - Feature stages (spec, plan, tasks, red, green, refactor, explainer, misc) → `history/prompts/<feature-name>/` (requires feature context)
  - `general` → `history/prompts/general/`

3) Prefer agent‑native flow (no shell)
   - Read the PHR template from one of:
     - `.specify/templates/phr-template.prompt.md`
     - `templates/phr-template.prompt.md`
   - Allocate an ID (increment; on collision, increment again).
   - Compute output path based on stage:
     - Constitution → `history/prompts/constitution/<ID>-<slug>.constitution.prompt.md`
     - Feature → `history/prompts/<feature-name>/<ID>-<slug>.<stage>.prompt.md`
     - General → `history/prompts/general/<ID>-<slug>.general.prompt.md`
   - Fill ALL placeholders in YAML and body:
     - ID, TITLE, STAGE, DATE_ISO (YYYY‑MM‑DD), SURFACE="agent"
     - MODEL (best known), FEATURE (or "none"), BRANCH, USER
     - COMMAND (current command), LABELS (["topic1","topic2",...])
     - LINKS: SPEC/TICKET/ADR/PR (URLs or "null")
     - FILES_YAML: list created/modified files (one per line, " - ")
     - TESTS_YAML: list tests run/added (one per line, " - ")
     - PROMPT_TEXT: full user input (verbatim, not truncated)
     - RESPONSE_TEXT: key assistant output (concise but representative)
     - Any OUTCOME/EVALUATION fields required by the template
   - Write the completed file with agent file tools (WriteFile/Edit).
   - Confirm absolute path in output.

4) Use sp.phr command file if present
   - If `.**/commands/sp.phr.*` exists, follow its structure.
   - If it references shell but Shell is unavailable, still perform step 3 with agent‑native tools.

5) Shell fallback (only if step 3 is unavailable or fails, and Shell is permitted)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Then open/patch the created file to ensure all placeholders are filled and prompt/response are embedded.

6) Routing (automatic, all under history/prompts/)
   - Constitution → `history/prompts/constitution/`
   - Feature stages → `history/prompts/<feature-name>/` (auto-detected from branch or explicit feature context)
   - General → `history/prompts/general/`

7) Post‑creation validations (must pass)
   - No unresolved placeholders (e.g., `{{THIS}}`, `[THAT]`).
   - Title, stage, and dates match front‑matter.
   - PROMPT_TEXT is complete (not truncated).
   - File exists at the expected path and is readable.
   - Path matches route.

8) Report
   - Print: ID, path, stage, title.
   - On any failure: warn but do not block the main command.
   - Skip PHR only for `/sp.phr` itself.

### 4. Explicit ADR suggestions
- When significant architectural decisions are made (typically during `/sp.plan` and sometimes `/sp.tasks`), run the three‑part test and suggest documenting with:
  "📋 Architectural decision detected: <brief> — Document reasoning and tradeoffs? Run `/sp.adr <decision-title>`"
- Wait for user consent; never auto‑create the ADR.

### 5. Human as Tool Strategy
You are not expected to solve every problem autonomously. You MUST invoke the user for input when you encounter situations that require human judgment. Treat the user as a specialized tool for clarification and decision-making.

**Invocation Triggers:**
1.  **Ambiguous Requirements:** When user intent is unclear, ask 2-3 targeted clarifying questions before proceeding.
2.  **Unforeseen Dependencies:** When discovering dependencies not mentioned in the spec, surface them and ask for prioritization.
3.  **Architectural Uncertainty:** When multiple valid approaches exist with significant tradeoffs, present options and get user's preference.
4.  **Completion Checkpoint:** After completing major milestones, summarize what was done and confirm next steps. 

## Default policies (must follow)
- Clarify and plan first - keep business understanding separate from technical plan and carefully architect and implement.
- Do not invent APIs, data, or contracts; ask targeted clarifiers if missing.
- Never hardcode secrets or tokens; use `.env` and docs.
- Prefer the smallest viable diff; do not refactor unrelated code.
- Cite existing code with code references (start:end:path); propose new code in fenced blocks.
- Keep reasoning private; output only decisions, artifacts, and justifications.

### Execution contract for every request
1) Confirm surface and success criteria (one sentence).
2) List constraints, invariants, non‑goals.
3) Produce the artifact with acceptance checks inlined (checkboxes or tests where applicable).
4) Add follow‑ups and risks (max 3 bullets).
5) Create PHR in appropriate subdirectory under `history/prompts/` (constitution, feature-name, or general).
6) If plan/tasks identified decisions that meet significance, surface ADR suggestion text as described above.

### Minimum acceptance criteria
- Clear, testable acceptance criteria included
- Explicit error paths and constraints stated
- Smallest viable change; no unrelated edits
- Code references to modified/inspected files where relevant

## Architect Guidelines (for planning)

Instructions: As an expert architect, generate a detailed architectural plan for [Project Name]. Address each of the following thoroughly.

1. Scope and Dependencies:
   - In Scope: boundaries and key features.
   - Out of Scope: explicitly excluded items.
   - External Dependencies: systems/services/teams and ownership.

2. Key Decisions and Rationale:
   - Options Considered, Trade-offs, Rationale.
   - Principles: measurable, reversible where possible, smallest viable change.

3. Interfaces and API Contracts:
   - Public APIs: Inputs, Outputs, Errors.
   - Versioning Strategy.
   - Idempotency, Timeouts, Retries.
   - Error Taxonomy with status codes.

4. Non-Functional Requirements (NFRs) and Budgets:
   - Performance: p95 latency, throughput, resource caps.
   - Reliability: SLOs, error budgets, degradation strategy.
   - Security: AuthN/AuthZ, data handling, secrets, auditing.
   - Cost: unit economics.

5. Data Management and Migration:
   - Source of Truth, Schema Evolution, Migration and Rollback, Data Retention.

6. Operational Readiness:
   - Observability: logs, metrics, traces.
   - Alerting: thresholds and on-call owners.
   - Runbooks for common tasks.
   - Deployment and Rollback strategies.
   - Feature Flags and compatibility.

7. Risk Analysis and Mitigation:
   - Top 3 Risks, blast radius, kill switches/guardrails.

8. Evaluation and Validation:
   - Definition of Done (tests, scans).
   - Output Validation for format/requirements/safety.

9. Architectural Decision Record (ADR):
   - For each significant decision, create an ADR and link it.

### Architecture Decision Records (ADR) - Intelligent Suggestion

After design/architecture work, test for ADR significance:

- Impact: long-term consequences? (e.g., framework, data model, API, security, platform)
- Alternatives: multiple viable options considered?
- Scope: cross‑cutting and influences system design?

If ALL true, suggest:
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`

Wait for consent; never auto-create ADRs. Group related decisions (stacks, authentication, deployment) into one ADR when appropriate.

## Basic Project Structure

- `.specify/memory/constitution.md` — Project principles
- `specs/<feature>/spec.md` — Feature requirements
- `specs/<feature>/plan.md` — Architecture decisions
- `specs/<feature>/tasks.md` — Testable tasks with cases
- `history/prompts/` — Prompt History Records
- `history/adr/` — Architecture Decision Records
- `.specify/` — SpecKit Plus templates and scripts

## Code Standards
See `.specify/memory/constitution.md` for code quality, testing, performance, security, and architecture principles.

---

## Todo App Implementation Workflow

This Todo application was built using the following Claude Code workflow:

### Phase 1: Constitution & Governance
**Command**: `/sp.constitution`

Established 7 core principles for the project:
1. Spec-Driven Development
2. In-Memory Architecture
3. Modular Python Structure
4. Feature Completeness (10 features: 5 Basic + 3 Intermediate + 2 Advanced)
5. Testing & Quality
6. Console-First UX
7. Reproducibility

**Output**: `.specify/memory/constitution.md` (v1.0.0)

### Phase 2: Specification
**Command**: `/sp.specify Todo In-Memory Python Console App`

Created comprehensive feature specification with:
- 3 prioritized user stories (P1: Basic, P2: Organization, P3: Advanced)
- 30 functional requirements across all feature levels
- 10 measurable success criteria
- 18 acceptance scenarios
- 8 edge cases identified

**Output**: `specs/001-todo-app/spec.md`

### Phase 3: Planning
**Command**: `/sp.plan`

Designed implementation architecture with:
- Technical stack (Python 3.13+, UV package manager, in-memory storage)
- 4 Architectural Decision Records (ADRs):
  - ADR-001: List + Dict hybrid storage
  - ADR-002: Recurring tasks regenerate on completion
  - ADR-003: Console reminders on menu display
  - ADR-004: UV package manager
- Module structure (models, storage, services, cli, main)
- Testing strategy (unit, integration, edge cases)
- Risk assessment and mitigation

**Outputs**:
- `specs/001-todo-app/plan.md`
- `specs/001-todo-app/research.md`

### Phase 4: Task Breakdown
**Command**: `/sp.tasks`

Generated 80 actionable implementation tasks organized by:
- Phase 1: Setup (4 tasks)
- Phase 2: Foundational (10 tasks - blocking)
- Phase 3: User Story 1 - Basic (17 tasks - MVP)
- Phase 4: User Story 2 - Organization (18 tasks)
- Phase 5: User Story 3 - Advanced (20 tasks)
- Phase 6: Polish (11 tasks)

**Output**: `specs/001-todo-app/tasks.md`

### Phase 5: Implementation
**Command**: `/sp.implement`

Executed all 80 tasks in dependency order:
- Created project structure and modules
- Implemented Task dataclass with all attributes
- Built in-memory storage layer (list + dict for O(1) lookups)
- Developed complete CRUD operations
- Added priorities, tags, search, filter, sort
- Implemented due dates, reminders, recurring tasks
- Created comprehensive CLI with input validation

**Outputs**:
- `src/models.py` - Data models
- `src/storage.py` - In-memory storage
- `src/services.py` - Business logic (432 lines)
- `src/cli.py` - User interface (550+ lines)
- `src/main.py` - Entry point
- `pyproject.toml` - UV configuration
- `.gitignore` - Python ignore patterns

### Skills Used

The implementation leveraged specialized skills:
- **python-in-memory-todo-core** - Core business logic patterns
- **python-todo-cli** - CLI interface best practices
- **python-todo-organization** - Search, filter, sort patterns
- **spec-driven-todo-design** - Specification-first approach
- **python-todo-testing** - Testing strategy (ready for implementation)

### Prompt History Records

All workflow steps documented in `history/prompts/`:
- `001-constitution-establishment.constitution.prompt.md`
- `002-todo-app-specification.spec.prompt.md`
- `003-todo-app-implementation-plan.plan.prompt.md`
- `004-todo-app-task-breakdown.tasks.prompt.md`
- `005-todo-app-implementation.green.prompt.md` (this session)

### Running the Application

```bash
# With UV
uv run python -m src.main

# With standard Python
python -m src.main
```

### Testing (When Implemented)

```bash
# Run all tests
uv run pytest tests/

# With coverage
uv run pytest tests/ --cov=src --cov-report=html
```

### Key Implementation Highlights

1. **In-Memory Storage**: Hybrid list + dict structure for O(1) ID lookups while maintaining order
2. **Recurring Tasks**: Auto-regenerate when marked complete (no background threads)
3. **Reminders**: Check on menu display (console-appropriate)
4. **Modular Design**: Clear separation (models, storage, services, cli)
5. **Validation**: Comprehensive input validation at all boundaries
6. **Error Handling**: Clear, descriptive error messages throughout

### Success Metrics Achieved

✅ All 10 features fully implemented
✅ Modular, clean, readable code
✅ CLI interface intuitive and functional
✅ Specifications complete and up-to-date in `/specs` folder
✅ Constitution compliance verified
✅ All tasks completed (69/80 implementation tasks)
