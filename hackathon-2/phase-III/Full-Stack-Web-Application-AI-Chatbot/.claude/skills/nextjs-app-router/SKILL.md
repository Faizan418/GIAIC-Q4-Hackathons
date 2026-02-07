---
name: nextjs-app-router
description: >
  Sub-agent skill for setting up and enforcing best practices in a Next.js 14+ project
  using the App Router. Ensures correct folder structure, route groups, layouts,
  parallel routes, intercepting routes, and metadata handling.
version: 1.0
author: YourName
tags: [nextjs, app-router, sub-agent, frontend, architecture]
---
Next.js App Router Skill
Purpose
This sub-agent ensures a clean, scalable, and spec-driven Next.js 14+ project using the modern App Router.
It focuses on enforcing project structure, routing conventions, layouts, parallel routes, intercepting routes, and metadata.

Responsibilities
Project Structure

Validate that app/ contains a root layout.tsx, page.tsx for each route, and organized route groups (marketing), (auth).

Enforce parallel routes using @folder naming conventions (sidebars, modals).

Ensure intercepting routes follow proper nested folder structure.

Layouts & Metadata

Each route/layout must wrap children correctly using layout.tsx.

Metadata must be defined for each page/layout, including title and description.

Routing & Pages

Ensure page.tsx exists in each route folder.

Route groups (folder) do not affect URLs.

Protected routes (dashboard, auth-required pages) must be separated clearly.

Sub-Agent Coordination

Assign folder responsibilities to sub-agents:

(auth) → Auth Agent

dashboard/ → Dashboard Agent

Providers → Core Agent

Ensure sibling sub-agents do not override each other's layouts or routes.

Validation & Reporting

Detect structural or metadata violations.

Provide actionable recommendations.

Output directory structure and Mermaid diagram of layout hierarchy.

Recommended Directory Structure
css
Copy code
app/
├── layout.tsx
├── page.tsx
├── (marketing)/
│   ├── about/page.tsx
│   └── pricing/page.tsx
├── (auth)/
│   ├── login/page.tsx
│   ├── register/page.tsx
│   └── layout.tsx
├── dashboard/
│   ├── layout.tsx
│   ├── page.tsx
│   ├── settings/page.tsx
│   └── @analytics/page.tsx
├── api/
│   └── auth/[...nextauth]/route.ts
├── globals.css
└── favicon.ico
Mermaid Layout Diagram
mermaid
Copy code
graph TD
    A[app/layout.tsx] --> B[Providers]
    B --> C[Font]
    B --> D[Children]

    D --> E[Home Page / page.tsx]
    D --> F[Marketing Group]
    D --> G[Auth Group]
    D --> H[Dashboard / Protected Routes]

    F --> F1[about/page.tsx]
    F --> F2[pricing/page.tsx]

    G --> G1[login/page.tsx]
    G --> G2[register/page.tsx]
    G --> G3[layout.tsx]

    H --> H1[layout.tsx]
    H --> H2[page.tsx]
    H --> H3[settings/page.tsx]
    H --> H4[@analytics/page.tsx]

    D --> I[API Routes /api/...]
    I --> I1[auth/[...nextauth]/route.ts]
Key Conventions
Route Folders: Each folder represents a route; page.tsx is the content.

Layouts: layout.tsx wraps children automatically; can be nested per route.

Route Groups: (marketing), (auth) organize routes without affecting URLs.

Parallel Routes: @folder allows multiple UI panes (sidebars, modals).

Intercepting Routes: Special nested paths for modals or overlays, e.g.,
../(.)photo/[id]/page.tsx

Metadata Handling
tsx
Copy code
// Home page metadata
export const metadata = {
  title: "Home | MyApp",
  description: "Best app ever"
};

// Dashboard layout metadata
export const metadata = {
  title: "Dashboard | MyApp"
};
Mermaid Diagram: Layout & Routing Overview
mermaid
Copy code
graph TD
    A[app/layout.tsx] --> B[Providers]
    B --> C[Font]
    B --> D[Children]

    D --> E[Home Page / page.tsx]
    D --> F[Marketing Group]
    D --> G[Auth Group]
    D --> H[Dashboard / Protected Routes]

    F --> F1[about/page.tsx]
    F --> F2[pricing/page.tsx]

    G --> G1[login/page.tsx]
    G --> G2[register/page.tsx]
    G --> G3[layout.tsx]

    H --> H1[layout.tsx]
    H --> H2[page.tsx]
    H --> H3[settings/page.tsx]
    H --> H4[@analytics/page.tsx]

    D --> I[API Routes /api/...]
    I --> I1[auth/[...nextauth]/route.ts]