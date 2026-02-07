---
name: end-to-end-flow-testing
description: Designs and implements comprehensive end-to-end tests covering critical user journeys using Playwright for browser automation and direct API calls for backend flows.
---

# End-to-End Flow Testing Skill

This skill ensures core user journeys work flawlessly from UI to database.

## Critical Flows to Test
- User registration → email verification → login → dashboard access
- Password reset flow
- Core feature CRUD (e.g., create post → view → edit → delete)
- Payment flow (if applicable)
- Admin actions

## Recommended Structure (Playwright)
e2e/
├── fixtures/
│   ├── authenticated-user.ts     # Login once, reuse session
│   └── test-data.ts              # Factories for users/posts
├── flows/
│   ├── registration.flow.ts
│   ├── login.flow.ts
│   ├── password-reset.flow.ts
│   └── core-feature.flow.ts
├── pages/
│   ├── login.page.ts
│   ├── dashboard.page.ts
│   └── settings.page.ts
├── tests/
│   ├── registration.spec.ts
│   ├── dashboard.spec.ts
│   └── admin.spec.ts
└── playwright.config.ts
text## Example Critical Flow (Mermaid)

```mermaid
sequenceDiagram
    participant User
    participant Browser
    participant Next.js
    participant FastAPI
    participant Neon DB

    User->>Browser: Visit /register
    Browser->>Next.js: Load page
    User->>Browser: Fill form + submit
    Browser->>Next.js: Server Action / API call
    Next.js->>FastAPI: POST /api/auth/register
    FastAPI->>Neon DB: Create user
    FastAPI->>Email: Send verification
    User->>Browser: Click verification link
    Browser->>Next.js: Verify token
    Next.js->>FastAPI: Mark verified
    User->>Browser: Login
    Browser->>Next.js: Set session
    Next.js->>Browser: Redirect to /dashboard
    Browser->>Next.js: Load dashboard data
    Next.js->>FastAPI: GET /api/user/me (authenticated)
    FastAPI->>Neon DB: Fetch user data
    FastAPI-->>Next.js: Return data
    Next.js-->>Browser: Render dashboard
Best Practices

Use Playwright test runner with tracing
Test in headed mode locally, headless in CI
Reuse authenticated sessions
Clean database state between tests
Assert UI + API + DB state