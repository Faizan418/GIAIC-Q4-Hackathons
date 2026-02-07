---
id: 1
title: "frontend-todo-app"
stage: "spec"
date_iso: "2026-01-11"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "frontend-todo-app"
branch: "1-frontend-todo-app"
user: "user"
command: "/sp.specify Frontend Specification for Phase II: Todo Full-Stack Web Application"
labels: ["frontend", "todo-app", "specification", "ui-ux"]
links:
  spec: "./specs/1-frontend-todo-app/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/1-frontend-todo-app/spec.md
  - specs/1-frontend-todo-app/checklists/requirements.md
tests_yaml: |
  - None
---

# Prompt History Record: Frontend Specification for Todo App

## Original Prompt
```
Frontend Specification for Phase II: Todo Full-Stack Web Application
Project Overview
This specification defines the frontend implementation for the Todo App in Phase II, transforming the console app into a modern, multi-user web application. Focus is exclusively on the frontend using Next.js 16+ with App Router, TypeScript, and Tailwind CSS. The frontend will integrate with a backend REST API (assumed to be implemented later) via JWT-authenticated calls. All features—basic, intermediate, and advanced—are required, ensuring a complete, responsive UI with user authentication. Development follows spec-driven approach: Write spec → Generate plan → Break into tasks → Implement via Claude Code. No manual coding.
Target Audience: End-users (individuals managing tasks) and developers reviewing the spec for implementation.
Focus: Build an intuitive, responsive interface supporting task CRUD, organization tools, and intelligent features. Emphasize usability with animations, dark theme (Black background, Blue accents, DarkGreen for completions), and accessibility (ARIA labels, keyboard navigation).
User Stories

As a user, I can sign up and log in to access my personal tasks securely.
As a user, I can create, view, update, delete, and mark tasks as complete.
As a user, I can assign priorities, tags, due dates, and set recurring tasks.
As a user, I can search, filter, and sort tasks for better organization.
As a user, I receive browser notifications for reminders.

Tech Stack (Frontend Only)

Framework: Next.js 16+ (App Router for routing, server/client components).
Language: TypeScript for type-safe code.
Styling: Tailwind CSS with custom theme (colors: black '#000000', blue '#007BFF', darkgreen '#006400'; animations via transitions and Framer Motion).
Authentication: Better Auth (with JWT plugin for token issuance).
API Client: Custom lib/api.ts for handling JWT-attached requests to backend endpoints.
Other: React-datepicker for date pickers, Notification API for reminders, Framer Motion for animations.

Folder Structure (Frontend)
text/frontend
├── app/                  # Pages and layouts
│   ├── login/page.tsx    # Login page
│   ├── register/page.tsx # Signup page
│   ├── dashboard/page.tsx# Main task dashboard
│   ├── tasks/[id]/page.tsx # Task detail/edit
│   └── layout.tsx        # Root layout with auth check
├── components/           # Reusable UI elements
│   ├── TaskForm.tsx      # Form for add/update
│   ├── TaskList.tsx      # List of tasks with cards
│   ├── TaskCard.tsx      # Individual task display
│   ├── SearchFilter.tsx  # Search bar and filters
│   ├── SortMenu.tsx      # Sorting dropdown
│   ├── Modal.tsx         # Confirmation dialogs
│   └── Loader.tsx        # Loading spinner
├── lib/                  # Utilities
│   └── api.ts            # API client with JWT
├── public/               # Static assets (icons)
├── tailwind.config.js    # Theme extensions
├── CLAUDE.md             # Frontend guidelines
└── next.config.js        # Next.js config
Authentication Spec

Use Better Auth for signup/login on /register and /login pages.
Forms: Email, password (required); name for signup.
On success: Issue JWT, store in session, redirect to /dashboard.
API Client: Attach 'Authorization: Bearer <token>' to all requests.
Protected Routes: Use middleware or layout to check auth; redirect unauth to /login.
Logout: Button in header, clear session.

UI Theme and Animations

Theme: Dark mode (black bg, white text); blue buttons/links; darkgreen for completed tasks.
Animations: Fade-in for new tasks (opacity 0->1, 300ms); slide-out for deletions (translateX); scale on button hovers (0.95->1); pulse for high-priority; stagger for list loads using Framer Motion's AnimatePresence.
Responsive: Mobile (stacked), Tablet/Desktop (grids). Use Tailwind breakpoints.

Pages Spec

/login and /register: Centered forms with inputs (blue borders on focus), submit button (blue bg). Error messages in red. Animation: Form fade-in on load.
/dashboard: Header (logo, user name, logout). Add Task form at top. TaskList below with search/filter/sort controls. Display counts (pending/completed). Animation: Loader spinner (blue) during fetches.
/tasks/[id]: Detail view with title, description, status, due date, etc. Edit form inline, delete button. Back to dashboard link. Modal for confirmations.

Components Spec

TaskForm: Inputs for title (required, 1-200 chars), description (optional, max 1000), due date (datepicker), priority (dropdown: high/red, medium/orange, low/green), tags (multi-select input), recurrence (dropdown: none/daily/weekly). Submit button. Validation: Show errors. Used in add/update.
TaskList: Ul or grid of TaskCards. Supports dynamic updates (e.g., after create/delete). Use server components for initial fetch, client for interactions.
TaskCard: Displays title (bold, strike if complete), description (truncate), status checkbox (darkgreen when checked), priority badge, tags as chips, due date. Edit/delete icons. Hover: Shadow lift.
SearchFilter: Search input (keyword), dropdowns for status (all/pending/completed), priority, date range. Apply button.
SortMenu: Dropdown for sort by (due date/priority/alpha, asc/desc).
Modal: Overlay for delete confirm, edit full-view. Close on esc/click outside.
Loader: Circular spinner for async operations.

Feature Implementation (Basic Level)

View Task List: On dashboard load, fetch GET /api/{user_id}/tasks via api client. Display in TaskList. Show title, status, created_at.
Add Task: Form submit → POST /api/{user_id}/tasks. Refresh list on success. Animation: New task fades in at top.
Update Task: Click edit on card → Inline form or modal with pre-filled data → PUT /api/{user_id}/tasks/{id}. Update list.
Delete Task: Icon click → Modal confirm → DELETE /api/{user_id}/tasks/{id}. Remove from list with slide-out.
Mark as Complete: Checkbox toggle → PATCH /api/{user_id}/tasks/{id}/complete. Update card (strike, color change).

Feature Implementation (Intermediate Level)

Priorities & Tags: Extend TaskForm and TaskCard. Priorities: High/medium/low with color badges. Tags: Array of strings (e.g., work/home). Store in task object.
Search & Filter: SearchFilter component → Append query params to GET (e.g., ?search=keyword&status=pending&priority=high). Re-fetch on apply.
Sort Tasks: SortMenu → Add ?sort=due_date asc to GET. Client-side sort as fallback if needed.

Feature Implementation (Advanced Level)

Due Dates & Reminders: DateTime picker in form. On create/update, if reminder set, use browser Notification API (request permission). Schedule via setTimeout based on due time.
Recurring Tasks: Recurrence selector in form. On mark complete, auto-create next instance (client-side logic or API call if backend supports). Display recurrence icon.

API Integration

All calls via lib/api.ts: Use fetch with JWT header. Handle errors (e.g., 401 redirect to login). User_id from decoded JWT.
Endpoints as specified: Assume backend filters by user.
Mocking: For development, use temporary mocks in api.ts until backend ready.

Acceptance Criteria

Responsive on all devices.
All features functional with API mocks.
Animations smooth, no performance lags.
Accessible: Screen reader friendly, contrast ratios met.
Error handling: Toasts for failures (e.g., "Task creation failed").
Security: No client-side data storage beyond session.

Success Criteria

User can complete full workflow (auth, CRUD, advanced features) without issues.
UI matches theme/animations as described.
Code type-safe, follows patterns (server-first, no inline styles).
All features implemented per user stories.

Constraints

Word count: This spec under 1000 (actual: 987).
No backend specs or implementation.
Use only specified stack; no additional libs without necessity.
Timeline: Implement after spec approval, via Claude Code.
Testing: Manual browser tests; add Jest if time allows.
```

## Response Summary
Created a comprehensive frontend specification for the Todo Full-Stack Web Application. The specification includes:

- Feature overview focusing on user needs and business value
- User stories covering all core functionality
- User scenarios and testing criteria
- Functional requirements for authentication, task management, organization features, and advanced features
- Non-functional requirements for performance, security, and compatibility
- Success criteria with measurable outcomes
- Key entities (User and Task)
- Assumptions about the system environment
- Dependencies on backend services
- Constraints for implementation

The specification was refined to remove implementation-specific details (technologies like Next.js, TypeScript, etc.) to maintain technology-agnosticism while preserving the essential requirements and user value propositions.

## Outcome
Successfully created a technology-agnostic frontend specification for the Todo application that focuses on user needs, functional requirements, and success criteria without prescribing specific implementation approaches.

## Evaluation
✅ User stories clearly defined
✅ Functional requirements are testable and unambiguous
✅ Success criteria are measurable and technology-agnostic
✅ Specification removes implementation details while preserving requirements
✅ All mandatory sections completed
✅ Ready for planning phase