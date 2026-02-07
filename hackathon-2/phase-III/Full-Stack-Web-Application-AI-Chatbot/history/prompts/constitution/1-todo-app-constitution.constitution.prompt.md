---
id: 1
title: "todo-app-constitution"
stage: "constitution"
date_iso: "2026-01-11"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "none"
branch: "master"
user: "user"
command: "/sp.constitution add my agents Project Constitution: Phase II - Todo Full-Stack Web Application"
labels: ["constitution", "setup", "spec-driven-development"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - .specify/memory/constitution.md
tests_yaml: |
  - None
---

# Prompt History Record: Todo Full-Stack Web Application Constitution

## Original Prompt
```
add my agents Project Constitution: Phase II - Todo Full-Stack Web Application
Project Overview
This constitution outlines the development framework for Phase II of the Todo Application, transforming a console-based app into a modern, multi-user full-stack web application. The project emphasizes spec-driven development using Claude Code and Spec-Kit Plus, ensuring no manual coding. We build on Phase I's completion, focusing on persistent storage, user authentication, and progressive features from basic CRUD to advanced intelligent functionalities. The app will support multiple users with isolated data, leveraging a monorepo structure for seamless integration.
Core Objective: Create an MVP todo app that is secure, scalable, and user-friendly, with a visually appealing UI themed in Black (#000000), Blue (#007BFF), and DarkGreen (#006400). Animations will enhance interactivity, such as smooth transitions for task additions/deletions and loading spinners.
Core Principles

Spec-Driven Development: All implementation starts with writing/updating specs in /specs folder, followed by Claude Code generation. No direct code edits; iterate via prompts.
Modularity and Reusability: Design components and APIs for extensibility, ensuring backend handles data logic and frontend focuses on presentation.
Security First: Enforce user isolation via JWT authentication; all data operations filtered by user ID.
User-Centric Design: Prioritize intuitive UI/UX with responsive design, accessibility (e.g., ARIA labels), and performance optimizations.
Efficiency: Use Agentic Dev Stack workflow: Spec → Plan → Tasks → Implement. Review prompts and iterations for quality.
Visual Consistency: Adhere to color scheme: Black for backgrounds/text, Blue for accents/buttons, DarkGreen for success states/completions. Incorporate CSS animations (e.g., fade-in for new tasks, slide-out for deletions) using Tailwind CSS transitions.

Key Standards

Development Workflow (Step-by-Step):
Spec Writing/Updating: Review requirements and update /specs files (e.g., @specs/features/task-crud.md for basic features). Use Spec-Kit conventions for structured markdown.
Generate Plan: Prompt Claude Code with "@specs/features/[feature].md generate implementation plan" to break into tasks (e.g., backend models, API endpoints, frontend components).
Task Implementation: For each task, prompt Claude Code (e.g., "Implement backend models per @specs/database/schema.md"). Handle frontend and backend separately via subfolder CLAUDE.md files.
Integration and Testing: Use docker-compose to run both services. Test APIs with tools like Postman; frontend with browser dev tools. Iterate by updating specs if issues arise.
Deployment Prep: Configure environment variables (e.g., BETTER_AUTH_SECRET, DATABASE_URL). Aim for Vercel (frontend) and Render/Heroku (backend) deployment.
Review and Merge: After Claude Code outputs, commit changes via Git. Ensure all features meet acceptance criteria before marking complete.

Project Structure (Monorepo):text
Full-Stack-Web-Application/
├── .spec-kit/                    # Spec-Kit config
│   └── config.yaml               # Defines structure, phases
├── specs/                        # Organized specifications
│   ├── overview.md               # Project summary
│   ├── architecture.md           # System design
│   ├── features/                 # Feature specs
│   │   ├── task-crud.md          # Basic/Intermediate/Advanced CRUD
│   │   ├── authentication.md     # Auth setup
│   │   └── advanced-features.md  # Recurring tasks, reminders
│   ├── api/                      # API details
│   │   └── rest-endpoints.md     # Endpoints spec
│   ├── database/                 # DB schema
│   │   └── schema.md             # Models and indexes
│   └── ui/                       # UI specs
│       ├── components.md         # Reusable components
│       └── pages.md              # Page layouts
├── CLAUDE.md                     # Root instructions for Claude Code
├── frontend/                     # Next.js app
│   ├── CLAUDE.md                 # Frontend guidelines
│   ├── app/                      # Pages and layouts
│   ├── components/               # UI elements (e.g., TaskCard, FormInput)
│   ├── lib/                      # Utilities (e.g., api.ts for API client)
│   ├── public/                   # Static assets
│   └── tailwind.config.js        # Custom colors/animations
├── backend/                      # FastAPI app
│   ├── CLAUDE.md                 # Backend guidelines
│   ├── main.py                   # App entry
│   ├── models.py                 # SQLModel models
│   ├── routes/                   # API handlers (e.g., tasks.py)
│   ├── db.py                     # DB connection
│   └── requirements.txt          # Dependencies
├── docker-compose.yml            # For local dev (Postgres, frontend, backend)
└── README.md                     # Setup instructionsThis structure allows Claude Code to reference specs easily (e.g., @specs/ui/pages.md) and edit code in context.
Technologies Defined:
Frontend: Next.js 16+ (App Router) for SSR/CSR, TypeScript for type safety, Tailwind CSS for styling with custom theme (extend colors: { black: '#000000', blue: '#007BFF', darkgreen: '#006400' }).
Backend: Python FastAPI for async APIs, SQLModel for ORM/models.
Database: Neon Serverless PostgreSQL for scalable, managed DB.
Authentication: Better Auth (with JWT plugin) for user sessions/tokens.
Other Tools: Docker for containerization, Git for version control, Claude Code + Spec-Kit Plus for generation.
Animations: Use Tailwind's transition utilities (e.g., transition-all duration-300 ease-in-out) for fades, slides, and hovers. Library: Framer Motion for complex animations (e.g., AnimatePresence for task list updates).
Environment: Shared secrets via .env files (e.g., BETTER_AUTH_SECRET for JWT).


Feature Definitions: CRUD Operations

Core CRUD (Basic Level):
These are foundational MVP features, implemented first for quick iteration.
Add Task (Create): User inputs title (required, 1-200 chars), description (optional, max 1000 chars). Backend: POST /api/{user_id}/tasks with Pydantic validation. Frontend: Form component with inputs, submit button (Blue bg, hover:darken). Animation: Fade-in new task in list.
Delete Task: Select task, confirm deletion. Backend: DELETE /api/{user_id}/tasks/{id}. Frontend: Trash icon button (Red on hover), modal confirmation. Animation: Slide-out and remove from DOM.
Update Task: Edit title/description. Backend: PUT /api/{user_id}/tasks/{id}. Frontend: Edit mode toggle, inline form. Animation: Smooth expand/collapse of edit fields.
View Task List: Display all user tasks. Backend: GET /api/{user_id}/tasks. Frontend: TaskList component as cards/ul, showing title, status, created_at. Animation: Loading spinner (circular, Blue).
Mark as Complete (Toggle): Checkbox or button to flip status. Backend: PATCH /api/{user_id}/tasks/{id}/complete. Frontend: Checkbox (DarkGreen when checked). Animation: Strike-through text with fade.

Intermediate CRUD (Organization & Usability):
Build on core for polish; implement after basics are tested.
Due Dates & Priorities: Add due_date (date picker), priority (high/medium/low dropdown, colors: Red/Orange/Green). Backend: Extend Task model, update CRUD endpoints. Frontend: Form additions, filter UI. Animation: Priority badges pulse on high.
Tags/Categories: Assign labels (e.g., work/home). Backend: Add tags array to model. Frontend: Multi-select input. Animation: Tag chips appear with scale-in.
Search & Filter: Keyword search, filters by status/priority/date. Backend: Query params (e.g., ?status=pending&priority=high). Frontend: Search bar (Blue border), dropdown filters. Animation: Filter results reload with stagger fade-in.
Sort Tasks: By due_date/priority/alpha. Backend: ?sort=due_date asc/desc. Frontend: Sort button/menu. Animation: List reorder with smooth transitions.

Advanced CRUD (Intelligent Features):
Add last for full functionality; require browser permissions for notifications.
Recurring Tasks: Set recurrence (daily/weekly). Backend: Add recurrence field, cron-like logic to auto-create on schedule (use background tasks). Frontend: Recurrence selector in form. Animation: Recurring icon spins on creation.
Time Reminders: Set deadlines with time; browser notifications. Backend: Store reminder_time. Frontend: DateTime picker (integrate react-datepicker), Notification API. Animation: Notification banner slides in (DarkGreen bg).


Full UI Definition
The UI is responsive (mobile-first), using Next.js App Router. Theme: Dark mode default (Black bg, white text), accents in Blue (links/buttons), DarkGreen (success/completions). Fonts: Inter (sans-serif). Layout: Sidebar nav (if multi-page), main content area.

Pages:
/login: Better Auth form (email/password, Blue submit button). Animation: Form fields focus glow.
/register: Similar to login, with name field.
/dashboard: Main page with TaskList, AddTask form at top. Search bar, filters/sort dropdowns. Animation: Tasks load with staggered entrance.
/tasks/[id]: Task detail view (edit form, delete button). Animation: Modal overlay fade.
Layout: Root layout with header (logo, user menu), footer (copyright). Use Next.js Layout component.

Components:
TaskCard: Card with title (bold, Blue if pending), description (truncate), status checkbox (DarkGreen), edit/delete icons. Hover: Shadow lift animation.
FormInput: Reusable input/textarea (Black bg, Blue border focus). Validation errors in Red.
Button: Variants: Primary (Blue bg, white text), Success (DarkGreen), Danger (Red). Animation: Scale on hover.
Modal: For confirmations/edits (centered, Black overlay). Animation: Scale-in from center.
Loader: Full-screen or inline spinner (Blue circles rotating).
TagChip: Rounded pills (colored by category). Animation: Remove with fade-out.
DatePicker: Integrated for due dates (calendar pop-up, DarkGreen selected days).

Styling and Animations:
Colors: Primary: Blue (#007BFF), Success: DarkGreen (#006400), Neutral: Black (#000000) for bgs, Gray (#6C757D) for text.
Animations: All via Tailwind/Framer: Fade (opacity 0 to 1, 300ms), Slide (translateX/Y), Scale (0.95 to 1 on hover), Pulse (for urgent tasks). Ensure 60fps performance; no heavy effects on mobile.
Accessibility: Alt text for icons, keyboard nav, screen reader support.
Responsive: Breakpoints: Mobile (<640px: stacked), Tablet (640-1024px: grid-2), Desktop (>1024px: grid-3 for tasks).


Constraints

Word Count Limit: This constitution is under 1800 words (current: 1487).
No Manual Coding: All code via Claude Code prompts.
Feature Phasing: Implement Basic → Intermediate → Advanced.
Testing: Unit (Pytest/Jest), Integration (API calls), E2E (Cypress if needed).
Dependencies: Minimal; no extra installs beyond stack.
Scalability: Design for 1000+ users; use indexing in DB.

Success Criteria

Functionality: All CRUDs work per specs; auth enforces isolation.
UI/UX: Responsive, themed correctly, animations smoot very very details maximum 1000 line
```

## Response Summary
Created a comprehensive constitution for the Todo Full-Stack Web Application project. The constitution includes:

- Core Principles: Spec-Driven Development, Modularity and Reusability, Security First, User-Centric Design, Efficiency, and Visual Consistency
- Key Standards: Development workflow, project structure, technology stack
- Feature Definitions: Core, Intermediate, and Advanced CRUD operations
- Full UI Definition: Pages, components, styling and animations
- Constraints and Success Criteria
- Governance rules

The constitution establishes the foundation for spec-driven development using Claude Code and Spec-Kit Plus, with a focus on creating a secure, scalable, and user-friendly todo application with a specific color theme and animations.

## Outcome
Successfully created a detailed project constitution that captures all the requirements specified by the user. The constitution serves as a guiding document for the development of the Todo Full-Stack Web Application, ensuring consistency in approach, technology stack, and design principles throughout the project lifecycle.

## Evaluation
The constitution addresses all the key aspects mentioned in the user's requirements:
✅ Spec-driven development approach
✅ Technology stack specifications
✅ Color scheme and UI guidelines
✅ Feature definitions for CRUD operations
✅ Project structure and workflow
✅ Security and user isolation requirements
✅ Animation and responsiveness requirements