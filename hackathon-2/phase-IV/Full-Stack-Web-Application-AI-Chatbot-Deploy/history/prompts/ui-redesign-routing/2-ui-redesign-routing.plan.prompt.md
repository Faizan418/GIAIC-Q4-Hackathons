---
id: 2
title: "ui-redesign-routing-plan"
stage: "plan"
date_iso: "2026-01-16"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "ui-redesign-routing"
branch: "1-ui-redesign-routing"
user: "user"

labels: ["plan", "ui", "routing", "authentication"]

links:
  spec: "specs/1-ui-redesign-routing/spec.md"
  ticket: null
  adr: null
  pr: null

files_yaml: |
  - specs/1-ui-redesign-routing/plan.md
  - specs/1-ui-redesign-routing/research.md
  - specs/1-ui-redesign-routing/data-model.md
  - specs/1-ui-redesign-routing/contracts.md
  - specs/1-ui-redesign-routing/quickstart.md

tests_yaml: []

command: "/sp.plan"

prompt_text: |
  # TASKAPP – sp.plan

  ## 1. Objective

  This execution plan defines **how** the requirements in `sp.specify.md` will be implemented. The goal is to:

  * Stabilize TASKAPP features
  * Remove all protected-route logic
  * Convert the application to a fully public-access model
  * Apply the approved UI redesign
  * Ensure a clean, build-safe Next.js implementation

  ---

  ## 2. Guiding Principles

  * **Build-first approach**: `npm run build` must pass at every phase
  * **No auth assumptions** in UI or routing
  * **Simple, explicit routing** over abstraction
  * **Incremental refactor**, not a full rewrite

  ---

  ## 3. Phase Breakdown

  ### Phase 1: Baseline Cleanup

  **Goal**: Remove blockers that cause runtime/build errors.

  Tasks:

  * Remove `middleware.ts` (or disable auth logic inside it)
  * Delete or comment out:

    * Session checks
    * Redirects based on auth
    * `useAuth`, `getSession`, `requireAuth` usage
  * Remove unused auth imports

  Deliverable:

  * App runs without auth dependency
  * No routing crashes

  ---

  ### Phase 2: Routing Refactor (Protected → Public)

  **Goal**: Make all routes publicly accessible.

  Tasks:

  * Audit all routes under `/app`
  * Ensure no page contains:

    * `redirect('/login')`
    * Conditional rendering tied to session
  * Normalize route structure:

    * `/dashboard`
    * `/tasks`
    * `/tasks/create`
    * `/tasks/[id]`

  Deliverable:

  * All routes open without login
  * Navigation works consistently

  ---

  ### Phase 3: UI Redesign Implementation

  **Goal**: Apply the dark-first TASKAPP design system.

  Tasks:

  * Implement global dark theme
  * Update layout shell:

    * Navbar
    * Page containers
  * Apply typography rules:

    * TASKAPP title styling
    * Text contrast fixes

  Deliverable:

  * Visual consistency across all pages
  * No mixed or broken themes

  ---

  ### Phase 4: Feature Stabilization (Tasks)

  **Goal**: Ensure core task features work reliably.

  Tasks:

  * Verify task list rendering
  * Fix create/edit/delete flows
  * Add empty and error states
  * Ensure UI updates correctly after actions

  Deliverable:

  * Tasks feature works end-to-end
  * No silent failures

  ---

  ### Phase 5: State & Data Safety

  **Goal**: Prevent UI crashes due to missing data.

  Tasks:

  * Add safe defaults for state
  * Guard against `undefined` values
  * Remove server-only assumptions

  Deliverable:

  * UI remains stable under all states

  ---

  ### Phase 6: Build & Quality Validation

  **Goal**: Ensure production readiness.

  Tasks:

  * Run `npm run build`
  * Resolve all warnings
  * Remove deprecated Next.js patterns
  * Verify no unused files remain

  Deliverable:

  * Clean production build

  ---

  ## 4. File-Level Action Map

  | Area                   | Action                      |
  | ---------------------- | --------------------------- |
  | middleware.ts          | Remove or disable           |
  | app/layout.tsx         | Apply global theme          |
  | app/page.tsx           | Public landing              |
  | app/dashboard/page.tsx | Public dashboard            |
  | components/Navbar.tsx  | Redesign, remove auth logic |
  | task components        | Remove session dependenc

response_text: |
  I'll help you create an implementation plan for the TASKAPP UI redesign and routing changes. Let me start by running the setup script to get the necessary context.

  [Initial setup and constitution reading output]

  Now I'll create the implementation plan file:

  [Plan creation output]

  Now I'll create the research document to resolve the unknowns:

  [Research document creation output]

  Now I'll create the data model document:

  [Data model creation output]

  Now I'll create the API contracts document:

  [Contracts document creation output]

  Now I'll create the quickstart guide:

  [Quickstart guide creation output]

  Now I'll update the implementation plan to reflect that the research and design phases are complete:

  [Plan update output]

  I have successfully completed the implementation plan for the TASKAPP UI redesign and routing changes. Here's what was accomplished:

  1. Created a comprehensive implementation plan with clear phases and tasks
  2. Conducted thorough research to resolve all unknowns about the current implementation
  3. Designed the data model for the local workspace functionality
  4. Defined API contracts for client-side operations
  5. Created a quickstart guide for setting up and using the application
  6. Updated the plan to reflect completed phases

  The implementation plan is organized into clear phases:
  - Phase 0: Research & Baseline Assessment (completed)
  - Phase 1: Design & Architecture (completed)
  - Phase 2: Implementation Roadmap (ready to begin)

  Three gates have been marked as complete:
  - GATE 1: Research complete - all unknowns resolved
  - GATE 2: Phase 0 - Research and baseline established
  - GATE 3: Phase 1 - Design and contracts complete

  The plan is now ready for the final approval gate before proceeding to task generation with /sp.tasks.

outcome: "Implementation plan created successfully with all research and design documents"
evaluation: "Successfully created comprehensive implementation plan, research, data model, contracts, and quickstart guide"
---
