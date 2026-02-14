# Implementation Plan: Frontend Todo App

## Technical Context

**Project**: Frontend Implementation for Phase II Todo Web Application
**Phase**: Frontend-only development (backend deferred)
**Current Date Reference**: January 11, 2026
**Development Paradigm**: Agentic + Spec-Driven (Claude Code + Spec-Kit Plus)

### Architecture Overview

**High-Level Architecture Sketch**
```
Full-Stack-Web-Application/ (monorepo root)
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
```

### Data Flow (Frontend → Future Backend)
User → Better Auth (session + JWT) → api client (interceptor adds Bearer) → fetch → assumed /api/{user_id}/tasks* endpoints
→ optimistic UI updates → revalidation (next.js cache) or SWR-like pattern

### Section Structure (Folder & Feature Organization)
Feature-first grouping inside components & hooks:
```
components/
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
```

Pages structure follows protected vs public routes:
- Public: /login, /register
- Protected: /dashboard, /tasks/[id]

### Architectural & Design Decisions

| Decision | Options Considered | Chosen | Tradeoffs / Rationale |
|----------|-------------------|---------|----------------------|
| State management | Zustand / Jotai / React Context / Server Components + fetch | Mostly Server Components + fetch + optimistic updates | Simplicity, no extra dependency, leverages Next.js cache. Downside: more manual optimistic handling. |
| Form handling | React Hook Form + Zod / Formik / native | React Hook Form + Zod | Best DX + validation + type-safety. Minimal boilerplate. |
| Data fetching pattern | SWR / TanStack Query / native fetch + use | Native fetch + revalidatePath / router.refresh | Zero dependencies, good enough for todo scale. Downside: less advanced caching strategies. |
| Authentication check | Middleware / Client-side redirect / Layout wrapper | Root layout server component + redirect() | Most secure & SEO friendly. Client check as fallback. |
| Animations library | Tailwind transitions only / Framer Motion | Framer Motion (minimal usage) + Tailwind | Smooth list animations (AnimatePresence) worth small dependency. |
| Date picker | react-datepicker / shadcn/ui calendar / native | shadcn/ui (if available) or react-datepicker | Consistent with potential future design system. |
| Toast notifications | sonner / react-hot-toast / custom | sonner (recommended with shadcn) | Beautiful, accessible, minimal config. |

### Agent Structure & Responsibilities
Agents living in @frontend/CLAUDE.md

| Agent Name | Primary Responsibility | Preferred Spec References | Output Style |
|------------|------------------------|---------------------------|--------------|
| claude/ui-ux-architect | Component hierarchy, naming, composition patterns | @specs/ui/components.md | Clean, atomic design suggestions |
| claude/database-architect | Data fetching, mutations, optimistic updates, error handling | @specs/features/task-crud.md, @specs/api/rest-endpoints.md | Sequence diagrams in text |
| claude/authentication-specialist | Authentication flows, protected routes, JWT handling | @specs/features/authentication.md | Security-first advice |
| claude/frontend-architect | Animations, loading states, micro-interactions, accessibility | Tailwind + Framer Motion patterns | Visual & accessibility focused |
| claude/form-master | Form validation, schema, UX for complex task creation | Zod + React Hook Form patterns | Schema-first |
| claude/feature-breaker | Breaks big features into smallest Claude-codeable tasks | All feature specs | Numbered task list |

## Constitution Check

Based on the project constitution, this plan must:
- Follow Spec-Driven Development principles (spec already created)
- Maintain modularity and reusability in component design
- Implement security-first approach with JWT authentication
- Prioritize user-centric design with responsive UI and accessibility
- Use efficient Agentic Dev Stack workflow (Spec → Plan → Tasks → Implement)
- Maintain visual consistency with black (#000000), blue (#007BFF), and darkgreen (#006400) color scheme

## Phase Gates

### Gate 1: Architecture Review
- [ ] Architecture follows Next.js 16+ App Router best practices
- [ ] Component structure supports reusability and maintainability
- [ ] Security measures implemented for authentication and data handling
- [ ] Tech stack decisions justified with tradeoffs documented

### Gate 2: Design Review
- [ ] Data model aligns with functional requirements
- [ ] API contracts defined for all required operations
- [ ] UI/UX design follows accessibility standards
- [ ] Animation and interaction patterns enhance user experience

## Phase 0: Research & Discovery

### Research Tasks

1. **Authentication Implementation Research**
   - Research Better Auth integration with Next.js App Router
   - Document JWT handling patterns in React applications
   - Identify best practices for session management

2. **Component Architecture Research**
   - Research atomic design patterns for React components
   - Identify best practices for component composition in Next.js
   - Document reusable UI component patterns

3. **Data Fetching Strategy Research**
   - Compare Next.js server components vs client components for data fetching
   - Research optimistic update patterns in React
   - Document error handling strategies for API calls

4. **Animation Implementation Research**
   - Research Framer Motion integration with Next.js
   - Document performance best practices for animations
   - Identify accessibility considerations for animations

5. **Form Handling Research**
   - Research React Hook Form integration with Zod validation
   - Document best practices for form state management
   - Identify accessibility patterns for forms

## Phase 1: Design & Architecture

### 1.1 Data Model Design

Based on the feature specification, the following entities will be implemented:

#### User Entity
- id: string (unique identifier)
- email: string (authentication credential)
- name: string (personal information)
- createdAt: Date
- updatedAt: Date

#### Task Entity
- id: string (unique identifier)
- title: string (1-200 characters)
- description: string (optional, up to 1000 characters)
- status: 'pending' | 'completed' (default: 'pending')
- priority: 'high' | 'medium' | 'low' (default: 'medium')
- tags: string[] (array of strings)
- dueDate: Date | null (optional)
- recurrencePattern: 'none' | 'daily' | 'weekly' (default: 'none')
- userId: string (foreign key to User)
- createdAt: Date
- updatedAt: Date

### 1.2 API Contract Design

The frontend will interact with the following assumed backend endpoints:

#### Authentication Endpoints
- `POST /api/auth/register` - User registration
- `POST /api/auth/login` - User login
- `POST /api/auth/logout` - User logout

#### Task Management Endpoints
- `GET /api/{user_id}/tasks` - Get all user tasks with query params for filtering/sorting
- `POST /api/{user_id}/tasks` - Create new task
- `PUT /api/{user_id}/tasks/{id}` - Update task
- `DELETE /api/{user_id}/tasks/{id}` - Delete task
- `PATCH /api/{user_id}/tasks/{id}/complete` - Toggle task completion

### 1.3 Component Architecture

#### UI Components (reusable primitives)
- Button: Various variants (primary, success, danger)
- Input: Text and textarea inputs with validation
- Modal: Confirmation dialogs and form overlays
- Card: Container for task display
- Badge: For priority and tags display

#### Task Domain Components
- TaskCard: Displays individual task with all details
- TaskForm: Handles task creation and editing
- TaskList: Container for multiple TaskCard components
- TaskStatusToggle: Checkbox for marking tasks as complete

#### Filter Components
- SearchInput: Keyword search with debouncing
- StatusFilter: Filter by task status
- PriorityFilter: Filter by priority level
- DateRangeFilter: Filter by due date range
- SortDropdown: Sort tasks by various criteria

#### Feedback Components
- Toast: For notifications and error messages
- Loader: For loading states
- ErrorBoundary: For error handling

## Phase 2: Implementation Strategy

### 2.1 Development Sequencing

#### Setup Phase
1. Initialize Next.js project with TypeScript
2. Configure Tailwind CSS with custom theme
3. Set up project structure and folder organization
4. Configure TypeScript with strict settings
5. Set up ESLint and Prettier for code quality

#### Authentication Implementation
1. Integrate Better Auth for user registration/login
2. Implement protected route patterns
3. Create login and registration pages
4. Set up JWT handling and session management

#### Core Task Functionality
1. Implement TaskList and TaskCard components
2. Create TaskForm for task creation and editing
3. Implement task CRUD operations
4. Add optimistic UI updates

#### Organization Features
1. Implement priority assignment and display
2. Add tag assignment and filtering
3. Create search functionality
4. Implement sorting capabilities

#### Advanced Features
1. Add due date functionality with date picker
2. Implement recurring tasks logic
3. Add browser notification support
4. Implement reminder system

#### UI/UX Enhancement
1. Add animations and transitions
2. Implement responsive design
3. Add accessibility features
4. Create loading and error states

### 2.2 Quality Validation Strategy

#### Type Safety
- tsc --noEmit must pass (CI check)
- Zod schemas for every form & API response

#### Unit / Hook Tests (nice to have, low priority)
- Vitest + @testing-library/react
- Priority: useTasks hook, optimistic update logic

#### Acceptance Criteria Checklist (manual + visual)
- Auth: Cannot access /dashboard without login, JWT attached, 401 → redirect
- Create: Required fields enforced, new task appears instantly (optimistic), animation correct
- Complete toggle: Instant UI update, checkbox state persisted, strike-through + darkgreen
- Delete: Slide out animation, item gone after confirm, no flicker
- Filter & Search: Debounced search, filters compose correctly, results update smoothly
- Priority & Tags: Visual badges/chips correct colors, filtering works
- Due Date & Reminder: Date picker UX good, browser notification permission requested when needed
- Recurring: UI shows recurrence badge, basic client-side next occurrence logic

#### Visual Regression (manual)
- Check mobile (375px), tablet (768px), desktop (1440px) for each major screen
- Dark theme only (no light mode in Phase II)

#### Performance Checklist
- No layout shift on task create/delete
- List with 50+ items still smooth (virtualize later if needed)
- Lighthouse score > 90 (Performance, Accessibility, Best Practices)

## Implementation Artifacts

### Generated Files
- `data-model.md` - Detailed data model specification
- `contracts/` - API contract definitions
- `quickstart.md` - Getting started guide for developers
- `frontend/CLAUDE.md` - Agent instructions for frontend development