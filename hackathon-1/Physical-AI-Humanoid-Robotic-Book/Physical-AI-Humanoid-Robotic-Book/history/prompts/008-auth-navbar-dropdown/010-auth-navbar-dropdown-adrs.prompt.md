---
id: 010
title: Auth Navbar Dropdown ADRs
stage: misc
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: auth-navbar-dropdown
branch: 007-user-auth-better-auth
user: Claude
command: /sp.adr
labels: ["auth", "navbar", "dropdown", "docusaurus", "adr", "architecture"]
links:
  spec: ../specs/008-auth-navbar-dropdown/spec.md
  ticket: null
  adr:
    - ../history/adr/0004-docusaurus-navbaritem-extension-for-authentication.md
    - ../history/adr/0005-better-auth-integration-for-session-management.md
  pr: null
files:
 - ../history/adr/0004-docusaurus-navbaritem-extension-for-authentication.md
 - ../history/adr/0005-better-auth-integration-for-session-management.md
tests:
 -

---

## Prompt

# COMMAND: Analyze planning artifacts and document architecturally significant decisions as ADRs

## CONTEXT

The user has completed feature planning and needs to:

- Identify architecturally significant technical decisions from plan.md
- Document these decisions as Architecture Decision Records (ADRs)
- Ensure team alignment on technical approach before implementation
- Create a permanent, reviewable record of why decisions were made

Architecture Decision Records capture decisions that:

- Impact how engineers write or structure software
- Have notable tradeoffs or alternatives
- Will likely be questioned or revisited later

**User's additional input:**

$ARGUMENTS

## YOUR ROLE

Act as a senior software architect with expertise in:

- Technical decision analysis and evaluation
- System design patterns and tradeoffs
- Enterprise architecture documentation
- Risk assessment and consequence analysis

## OUTPUT STRUCTURE (with quick flywheel hooks)

Execute this workflow in 6 sequential steps. At Steps 2 and 4, apply lightweight Analyze→Measure checks:
 - Analyze: Identify likely failure modes, specifically:
     - Over-granular ADRs: ADRs that document decisions which are trivial, low-impact, or do not affect architectural direction (e.g., naming conventions, minor refactorings).
     - Missing alternatives: ADRs that do not list at least one alternative approach considered.
 - Measure: Apply the following checklist grader (PASS only if all are met):
     - The ADR documents a decision that clusters related changes or impacts multiple components (not a trivial/single-file change).
     - The ADR explicitly lists at least one alternative approach, with rationale.
     - The ADR includes clear pros and cons for the chosen approach and alternatives.
     - The ADR is concise but sufficiently detailed for future reference.

## Step 1: Load Planning Context

Run `.specify/scripts/bash/check-prerequisites.sh --json` from repo root and parse JSON for FEATURE_DIR and AVAILABLE_DOCS.

Derive absolute paths:

- PLAN = FEATURE_DIR/plan.md (REQUIRED - abort if missing with "Run /sp.plan first")
- RESEARCH = FEATURE_DIR/research.md (if exists)
- DATA_MODEL = FEATURE_DIR/data-model.md (if exists)
- CONTRACTS_DIR = FEATURE_DIR/contracts/ (if exists)

## Step 2: Extract Architectural Decisions (Analyze)

Load plan.md and available artifacts. Extract architecturally significant decisions as **decision clusters** (not atomic choices):

**✅ GOOD (Clustered):**

- "Frontend Stack" (Next.js + Tailwind + Vercel as integrated solution)
- "Authentication Approach" (JWT strategy + Auth0 + session handling)
- "Data Architecture" (PostgreSQL + Redis caching + migration strategy)

**❌ BAD (Over-granular):**

- Separate ADRs for Next.js, Tailwind, and Vercel
- Separate ADRs for each library choice

**Clustering Rules:**

- Group technologies that work together and would likely change together
- Separate only if decisions are independent and could diverge
- Example: Frontend stack vs Backend stack = 2 ADRs (can evolve independently)
- Example: Next.js + Tailwind + Vercel = 1 ADR (integrated, change together)

For each decision cluster, note: what was decided, why, where in docs.

## Step 3: Check Existing ADRs

Scan `history/adr/` directory. For each extracted decision:

- If covered by existing ADR → note reference
- If conflicts with existing ADR → flag conflict
- If not covered → mark as ADR candidate

## Step 4: Apply Significance Test (Measure)

For each ADR candidate, test:

- Does it impact how engineers write/structure software?
- Are there notable tradeoffs or alternatives?
- Will it be questioned or revisited later?

Only proceed with ADRs that pass ALL three tests.

## Step 5: Create ADRs (Improve)

For each qualifying decision cluster:

1. Generate concise title reflecting the cluster (e.g., "Frontend Technology Stack" not "Use Next.js")
2. Run `create-adr.sh "<title>"` from repo root
3. Parse JSON response for `adr_path` and `adr_id`
4. Read created file (contains template with {{PLACEHOLDERS}})
5. Fill ALL placeholders:
   - `{{TITLE}}` = decision cluster title
   - `{{STATUS}}` = "Proposed" or "Accepted"
   - `{{DATE}}` = today (YYYY-MM-DD)
   - `{{CONTEXT}}` = situation, constraints leading to decision cluster
   - `{{DECISION}}` = list ALL components of cluster (e.g., "Framework: Next.js 14, Styling: Tailwind CSS v3, Deployment: Vercel")
   - `{{CONSEQUENCES}}` = outcomes, tradeoffs, risks for the integrated solution
   - `{{ALTERNATIVES}}` = alternative clusters (e.g., "Remix + styled-components + Cloudflare")
   - `{{REFERENCES}}` = plan.md, research.md, data-model.md
6. Save file

## Step 6: Report Completion

Output:

```
✅ ADR Review Complete - Created N ADRs, referenced M existing
```

List created ADRs with ID and title.

If conflicts detected:

```
⚠️ Conflicts with existing ADRs [IDs]. Review and update outdated decisions or revise plan.
```

If create-adr.sh fails: Report script error and skip that ADR.

## FORMATTING REQUIREMENTS

Present results in this exact structure:

```
✅ ADR Review Complete
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📋 Created ADRs: {count}
   - ADR-{id}: {title}
   - ADR-{id}: {title}

📚 Referenced Existing: {count}
   - ADR-{id}: {title}

⚠️  Conflicts Detected: {count}
   - ADR-{id}: {conflict description}

Next Steps:
→ Resolve conflicts before proceeding to /sp.tasks
→ Review created ADRs with team
→ Update plan.md if needed

Acceptance Criteria (PASS only if all true)
- Decisions are clustered (not atomic), with explicit alternatives and tradeoffs
- Consequences cover both positive and negative outcomes
- References link back to plan and related docs
```

## ERROR HANDLING

If plan.md missing:

- Display: "❌ Error: plan.md not found. Run /sp.plan first to generate planning artifacts."
- Exit gracefully without creating any ADRs

If create-adr.sh fails:

- Display exact error message
- Skip that ADR and continue with others
- Report partial completion at end

## TONE

Be thorough, analytical, and decision-focused. Emphasize the "why" behind each decision and its long-term implications.

## Response snapshot

✅ ADR Review Complete
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📋 Created ADRs: 2
   - ADR-0004: Docusaurus NavbarItem Extension for Authentication
   - ADR-0005: Better Auth Integration for Session Management

📚 Referenced Existing: 0

⚠️  Conflicts Detected: 0

Next Steps:
→ Review created ADRs with team
→ Update plan.md if needed

Acceptance Criteria (PASS only if all true)
- Decisions are clustered (not atomic), with explicit alternatives and tradeoffs
- Consequences cover both positive and negative outcomes
- References link back to plan and related docs

## Outcome

- ✅ Impact: Created 2 architecture decision records for the auth-aware navbar dropdown feature
- 🧪 Tests: No tests run yet, ADRs created for documentation and decision tracking
- 📁 Files: Created ADR-0004 and ADR-0005 in history/adr directory
- 🔁 Next prompts: Ready for task generation phase with /sp.tasks
- 🧠 Reflection: ADRs document important architectural decisions for long-term maintainability
- 📝 Decisions clustered appropriately with alternatives and consequences

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable at ADR creation stage
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Proceed to task generation phase
