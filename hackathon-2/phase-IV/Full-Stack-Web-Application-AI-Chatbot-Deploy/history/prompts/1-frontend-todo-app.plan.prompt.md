---
id: 1
title: "frontend-todo-app"
stage: "plan"
date_iso: "2026-01-11"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "frontend-todo-app"
branch: "1-frontend-todo-app"
user: "user"
command: "/sp.plan sp.plan
Project: Frontend Implementation for Phase II Todo Web Application
Phase: Frontend-only development (backend deferred)
Current Date Reference: January 11, 2026
Development Paradigm: Agentic + Spec-Driven (Claude Code + Spec-Kit Plus)
1. High-Level Architecture Sketch
texthackathon-todo/ (monorepo root)
└── frontend/                      ← only active folder for this phase
    ├── app/                       # App Router pages & layouts
    │   ├── (auth)/                # login & register (group route - no layout)
    │   ├── dashboard/             # protected main workspace
    │   ├── tasks/[id]/            # dynamic task detail
    │   ├── layout.tsx             # root layout + auth guard
    │   └── page.tsx               # redirect / → login or dashboard
    ├── components/                # atomic → organism hierarchy
    │   ├── ui/                    # primitives (Button, Input, Modal, Card, Badge, etc.)
    │   ├── task/                  # domain: TaskCard, TaskForm, TaskList
    │   ├── filters/               # SearchFilter, SortMenu, PriorityFilter
    │   └── layout/                # Header, Footer, Sidebar (future)
    ├── lib/
    │   ├── api/                   # typed API client + interceptors (JWT)
    │   ├── auth/                  # better-auth hooks & utils
    │   └── hooks/                 # useTasks, useDebounce, etc.
    ├── hooks/                     # custom React hooks
    ├── types/                     # shared TypeScript interfaces (Task, User, etc.)
    ├── styles/                    # global.css + tailwind config extensions
    ├── public/                    # static assets
    └── CLAUDE.md                  # agent instructions specific to frontend
Data Flow (Frontend → Future Backend)
User → Better Auth (session + JWT) → api client (interceptor adds Bearer) → fetch → assumed /api/{user_id}/tasks* endpoints
→ optimistic UI updates → revalidation (next.js cache) or SWR-like pattern
2. Section Structure (Folder & Feature Organization)
Feature-first grouping inside components & hooks:
textcomponents/
├── ui/                     # reusable primitives
├── task/
│   ├── TaskCard/
│   ├── TaskForm/
│   ├── TaskList/
│   └── TaskStatusToggle/
├── filters/
│   ├── SearchInput/
│   ├── StatusFilter/
│   ├── PriorityFilter/
│   ├── DateRangeFilter/
│   └── SortDropdown/
└── feedback/
    ├── Toast/
    ├── Loader/
    └── ErrorBoundary/
Pages structure follows protected vs public routes:

Public: /login, /register
Protected: /dashboard, /tasks/[id]

3. Research & Reference Approach (concurrent)
We do not do big upfront research. Instead we use:

Quick spec reference reading before each task
Claude Code context injection via @specs/ui/... & @frontend/CLAUDE.md
Targeted questions to Claude when pattern decision is unclear
Live documentation update when new pattern is discovered

Reference order priority (always read in this sequence):

@specs/ui/components.md
@specs/ui/pages.md
@frontend/CLAUDE.md
@specs/features/task-crud.md (for feature acceptance criteria)
Tailwind docs + Next.js App Router patterns (mental model)

4. Important Architectural & Design Decisions (with tradeoffs)





























































#DecisionOptions ConsideredChosenTradeoffs / Rationale1State managementZustand / Jotai / React Context / Server Components + fetchMostly Server Components + fetch + optimistic updatesSimplicity, no extra dependency, leverages Next.js cache. Downside: more manual optimistic handling.2Form handlingReact Hook Form + Zod / Formik / nativeReact Hook Form + ZodBest DX + validation + type-safety. Minimal boilerplate.3Data fetching patternSWR / TanStack Query / native fetch + useNative fetch + revalidatePath / router.refreshZero dependencies, good enough for todo scale. Downside: less advanced caching strategies.4Authentication checkMiddleware / Client-side redirect / Layout wrapperRoot layout server component + redirect()Most secure & SEO friendly. Client check as fallback.5Animations libraryTailwind transitions only / Framer MotionFramer Motion (minimal usage) + TailwindSmooth list animations (AnimatePresence) worth small dependency.6Date pickerreact-datepicker / shadcn/ui calendar / nativeshadcn/ui (if available) or react-datepickerConsistent with potential future design system.7Toast notificationssonner / react-hot-toast / customsonner (recommended with shadcn)Beautiful, accessible, minimal config.
5. Agent Structure & Responsibilities
@agents mansions – named agents living in @frontend/CLAUDE.md















































Agent NamePrimary ResponsibilityPreferred Spec ReferencesOutput Styleclaude/ui-architectComponent hierarchy, naming, composition patterns@specs/ui/components.mdClean, atomic design suggestionsclaude/data-flowData fetching, mutations, optimistic updates, error handling@specs/features/task-crud.md, @specs/api/rest-endpoints.mdSequence diagrams in textclaude/auth-guardianAuthentication flows, protected routes, JWT handling@specs/features/authentication.mdSecurity-first adviceclaude/ux-polisherAnimations, loading states, micro-interactions, accessibilityTailwind + Framer Motion patternsVisual & accessibility focusedclaude/form-masterForm validation, schema, UX for complex task creationZod + React Hook Form patternsSchema-firstclaude/feature-breakerBreaks big features into smallest Claude-codeable tasksAll feature specsNumbered task list
6. Testing & Quality Validation Strategy
Validation layers (in order of execution):

Type Safety
tsc --noEmit must pass (CI check)
Zod schemas for every form & API response

Unit / Hook Tests (nice to have, low priority)
Vitest + @testing-library/react
Priority: useTasks hook, optimistic update logic

Acceptance Criteria Checklist (manual + visual)










































FeatureManual Validation ChecksAuthCannot access /dashboard without login, JWT attached, 401 → redirectCreateRequired fields enforced, new task appears instantly (optimistic), animation correctComplete toggleInstant UI update, checkbox state persisted, strike-through + darkgreenDeleteSlide out animation, item gone after confirm, no flickerFilter & SearchDebounced search, filters compose correctly, results update smoothlyPriority & TagsVisual badges/chips correct colors, filtering worksDue Date & ReminderDate picker UX good, browser notification permission requested when neededRecurringUI shows recurrence badge, basic client-side next occurrence logic

Visual Regression (manual)
Check mobile (375px), tablet (768px), desktop (1440px) for each major screen
Dark theme only (no light mode in Phase II)

Performance Checklist
No layout shift on task create/delete
List with 50+ items still smooth (virtualize later if needed)
Lighthouse score > 90 (Performance, Accessibility, Best Practices)


7. Development Sequencing (recommended order)

Setup"
labels: ["frontend", "todo-app", "planning", "architecture"]
links:
  spec: "./specs/1-frontend-todo-app/spec.md"
  plan: "./specs/1-frontend-todo-app/plan.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/1-frontend-todo-app/plan.md
  - specs/1-frontend-todo-app/research.md
  - specs/1-frontend-todo-app/data-model.md
  - specs/1-frontend-todo-app/contracts/api-contracts.md
  - specs/1-frontend-todo-app/quickstart.md
  - frontend/CLAUDE.md
tests_yaml: |
  - None
---

# Prompt History Record: Frontend Todo App Implementation Plan

## Original Prompt
```
sp.plan
Project: Frontend Implementation for Phase II Todo Web Application
Phase: Frontend-only development (backend deferred)
Current Date Reference: January 11, 2026
Development Paradigm: Agentic + Spec-Driven (Claude Code + Spec-Kit Plus)
1. High-Level Architecture Sketch
texthackathon-todo/ (monorepo root)
└── frontend/                      ← only active folder for this phase
    ├── app/                       # App Router pages & layouts
    │   ├── (auth)/                # login & register (group route - no layout)
    │   ├── dashboard/             # protected main workspace
    │   ├── tasks/[id]/            # dynamic task detail
    │   ├── layout.tsx             # root layout + auth guard
    │   └── page.tsx               # redirect / → login or dashboard
    ├── components/                # atomic → organism hierarchy
    │   ├── ui/                    # primitives (Button, Input, Modal, Card, Badge, etc.)
    │   ├── task/                  # domain: TaskCard, TaskForm, TaskList
    │   ├── filters/               # SearchFilter, SortMenu, PriorityFilter
    │   └── layout/                # Header, Footer, Sidebar (future)
    ├── lib/
    │   ├── api/                   # typed API client + interceptors (JWT)
    │   ├── auth/                  # better-auth hooks & utils
    │   └── hooks/                 # useTasks, useDebounce, etc.
    ├── hooks/                     # custom React hooks
    ├── types/                     # shared TypeScript interfaces (Task, User, etc.)
    ├── styles/                    # global.css + tailwind config extensions
    ├── public/                    # static assets
    └── CLAUDE.md                  # agent instructions specific to frontend
Data Flow (Frontend → Future Backend)
User → Better Auth (session + JWT) → api client (interceptor adds Bearer) → fetch → assumed /api/{user_id}/tasks* endpoints
→ optimistic UI updates → revalidation (next.js cache) or SWR-like pattern
2. Section Structure (Folder & Feature Organization)
Feature-first grouping inside components & hooks:
textcomponents/
├── ui/                     # reusable primitives
├── task/
│   ├── TaskCard/
│   ├── TaskForm/
│   ├── TaskList/
│   └── TaskStatusToggle/
├── filters/
│   ├── SearchInput/
│   ├── StatusFilter/
│   ├── PriorityFilter/
│   ├── DateRangeFilter/
│   └── SortDropdown/
└── feedback/
    ├── Toast/
    ├── Loader/
    └── ErrorBoundary/
Pages structure follows protected vs public routes:

Public: /login, /register
Protected: /dashboard, /tasks/[id]

3. Research & Reference Approach (concurrent)
We do not do big upfront research. Instead we use:

Quick spec reference reading before each task
Claude Code context injection via @specs/ui/... & @frontend/CLAUDE.md
Targeted questions to Claude when pattern decision is unclear
Live documentation update when new pattern is discovered

Reference order priority (always read in this sequence):

@specs/ui/components.md
@specs/ui/pages.md
@frontend/CLAUDE.md
@specs/features/task-crud.md (for feature acceptance criteria)
Tailwind docs + Next.js App Router patterns (mental model)

4. Important Architectural & Design Decisions (with tradeoffs)





























































#DecisionOptions ConsideredChosenTradeoffs / Rationale1State managementZustand / Jotai / React Context / Server Components + fetchMostly Server Components + fetch + optimistic updatesSimplicity, no extra dependency, leverages Next.js cache. Downside: more manual optimistic handling.2Form handlingReact Hook Form + Zod / Formik / nativeReact Hook Form + ZodBest DX + validation + type-safety. Minimal boilerplate.3Data fetching patternSWR / TanStack Query / native fetch + useNative fetch + revalidatePath / router.refreshZero dependencies, good enough for todo scale. Downside: less advanced caching strategies.4Authentication checkMiddleware / Client-side redirect / Layout wrapperRoot layout server component + redirect()Most secure & SEO friendly. Client check as fallback.5Animations libraryTailwind transitions only / Framer MotionFramer Motion (minimal usage) + TailwindSmooth list animations (AnimatePresence) worth small dependency.6Date pickerreact-datepicker / shadcn/ui calendar / nativeshadcn/ui (if available) or react-datepickerConsistent with potential future design system.7Toast notificationssonner / react-hot-toast / customsonner (recommended with shadcn)Beautiful, accessible, minimal config.
5. Agent Structure & Responsibilities
@agents mansions – named agents living in @frontend/CLAUDE.md















































Agent NamePrimary ResponsibilityPreferred Spec ReferencesOutput Styleclaude/ui-architectComponent hierarchy, naming, composition patterns@specs/ui/components.mdClean, atomic design suggestionsclaude/data-flowData fetching, mutations, optimistic updates, error handling@specs/features/task-crud.md, @specs/api/rest-endpoints.mdSequence diagrams in textclaude/auth-guardianAuthentication flows, protected routes, JWT handling@specs/features/authentication.mdSecurity-first adviceclaude/ux-polisherAnimations, loading states, micro-interactions, accessibilityTailwind + Framer Motion patternsVisual & accessibility focusedclaude/form-masterForm validation, schema, UX for complex task creationZod + React Hook Form patternsSchema-firstclaude/feature-breakerBreaks big features into smallest Claude-codeable tasksAll feature specsNumbered task list
6. Testing & Quality Validation Strategy
Validation layers (in order of execution):

Type Safety
tsc --noEmit must pass (CI check)
Zod schemas for every form & API response

Unit / Hook Tests (nice to have, low priority)
Vitest + @testing-library/react
Priority: useTasks hook, optimistic update logic

Acceptance Criteria Checklist (manual + visual)










































FeatureManual Validation ChecksAuthCannot access /dashboard without login, JWT attached, 401 → redirectCreateRequired fields enforced, new task appears instantly (optimistic), animation correctComplete toggleInstant UI update, checkbox state persisted, strike-through + darkgreenDeleteSlide out animation, item gone after confirm, no flickerFilter & SearchDebounced search, filters compose correctly, results update smoothlyPriority & TagsVisual badges/chips correct colors, filtering worksDue Date & ReminderDate picker UX good, browser notification permission requested when neededRecurringUI shows recurrence badge, basic client-side next occurrence logic

Visual Regression (manual)
Check mobile (375px), tablet (768px), desktop (1440px) for each major screen
Dark theme only (no light mode in Phase II)

Performance Checklist
No layout shift on task create/delete
List with 50+ items still smooth (virtualize later if needed)
Lighthouse score > 90 (Performance, Accessibility, Best Practices)


7. Development Sequencing (recommended order)

Setup
```

## Response Summary
Created a comprehensive implementation plan for the frontend todo app based on the provided specifications. The plan includes:

- Detailed technical context with architecture overview
- Research summary with key technology decisions
- Data model specification with TypeScript interfaces
- API contracts for all required endpoints
- Quickstart guide for developers
- Claude Code instructions for frontend development

The plan addresses all architectural decisions mentioned in the prompt, including authentication (Better Auth), form handling (React Hook Form + Zod), data fetching (Server Components + fetch), animations (Framer Motion + Tailwind), and component architecture (atomic design).

## Outcome
Successfully created a complete implementation plan with all necessary supporting documents for the frontend todo application. The plan follows the spec-driven development approach and provides clear guidance for implementation.

## Evaluation
✅ Implementation plan created with technical context
✅ Research document completed with technology decisions
✅ Data model specified with TypeScript interfaces
✅ API contracts defined for all endpoints
✅ Quickstart guide created for developers
✅ Claude Code instructions configured for frontend
✅ All required artifacts generated and properly linked