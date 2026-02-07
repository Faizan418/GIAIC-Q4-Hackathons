---
id: 002
title: Auth Onboarding Constitution
stage: constitution
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, auth, onboarding, better-auth, nextjs, personalization]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - ../../.specify/memory/constitution.md
tests:
 - null
---

## Prompt

main constituion.md file inthe add before content not remove  # sp.constitution — Auth & Onboarding Specification

## 1. Purpose

Define the **Signup & Signin system using Better Auth**, including **user background profiling** (software + hardware) and **custom animated UI styles** for forms and buttons. This document acts as a binding implementation and UX standard.

---

## 2. Authentication Stack

* Framework: **Next.js (App Router) + TypeScript**
* Auth Provider: **Better Auth**
* Session Strategy: Cookie-based sessions
* Profile Storage: Application database (separate from auth)

Auth is responsible **only for identity**. Personalization data is handled by the application.

---

## 3. Signup & Signin Flow

### Signup Flow

1. User enters email + password
2. Account created via Better Auth
3. User is redirected to onboarding
4. Background questions collected
5. Profile persisted
6. Redirect to dashboard

### Signin Flow

1. User authenticates
2. Session restored
3. If profile missing → onboarding
4. Else → dashboard

---

## 4. User Background Questions (Mandatory)

### Software Background

* Skill level: BEGINNER | INTERMEDIATE | ADVANCED
* Known languages: Python, JavaScript, C++, Other

### Hardware Background

* Electronics experience: NONE | BASIC | INTERMEDIATE | ADVANCED
* Boards used: Arduino, ESP32, Raspberry Pi

### Learning Intent

* Track:

  * SOFTWARE_ONLY
  * HARDWARE_ONLY
  * FULL_ROBOTICS

This data is used to personalize content visibility and sequencing.

---

## 5. Better Auth Configuration

```ts
// lib/auth.ts
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  emailAndPassword: {
    enabled: true,
  },
  session: {
    cookieCache: true,
  },
});
```

---

## 6. Signup Page (Logic Only)

```tsx
import { signUp } from "better-auth/react";

export async function signup(formData: FormData) {
  await signUp.email({
    email: formData.get("email") as string,
    password: formData.get("password") as string,
    callbackURL: "/onboarding",
  });
}
```

---

## 7. Signin Page (Logic Only)

```tsx
import { signIn } from "better-auth/react";

export async function signin(formData: FormData) {
  await signIn.email({
    email: formData.get("email") as string,
    password: formData.get("password") as string,
    callbackURL: "/dashboard",
  });
}
```

---

## 8. Animated UI Constitution (CSS)

### Design Principles

* Minimal
* Motion-driven feedback
* No third-party animation libraries
* Hardware-accelerated transforms only

### Form Container Animation

```css
.auth-card {
  background: rgba(255, 255, 255, 0.06);
  backdrop-filter: blur(14px);
  border-radius: 16px;
  padding: 2rem;
  animation: cardEnter 600ms ease-out both;
}

@keyframes cardEnter {
  from {
    opacity: 0;
    transform: translateY(24px) scale(0.96);
  }
  to {
    opacity: 1;
    transform: translateY(0) scale(1);
  }
}
```

---

### Input Field Animation

```css
.auth-input {
  width: 100%;
  padding: 12px 14px;
  border-radius: 10px;
  border: 1px solid rgba(255,255,255,0.12);
  background: rgba(0,0,0,0.3);
  color: #fff;
  transition: border 200ms ease, box-shadow 200ms ease;
}

.auth-input:focus {
  outline: none;
  border-color: #4fd1c5;
  box-shadow: 0 0 0 3px rgba(79,209,197,0.25);
}
```

---

### Button Animation (Signup / Signin)

```css
.auth-button {
  width: 100%;
  padding: 14px;
  border-radius: 12px;
  font-weight: 600;
  background: linear-gradient(135deg, #4fd1c5, #667eea);
  color: #000;
  border: none;
  cursor: pointer;
  transition: transform 150ms ease, box-shadow 150ms ease;
}

.auth-button:hover {
  transform: translateY(-2px);
  box-shadow: 0 12px 30px rgba(79,209,197,0.35);
}

.auth-button:active {
  transform: scale(0.96);
}
```

---

### Loading State Animation

```css
.auth-button.loading {
  pointer-events: none;
  opacity: 0.8;
}

.auth-button.loading::after {
  content: "";
  width: 18px;
  height: 18px;
  border: 2px solid rgba(0,0,0,0.3);
  border-top-color: #000;
  border-radius: 50%;
  animation: spin 700ms linear infinite;
  display: inline-block;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}
```

---

## 9. Enforcement Rules

* Signup and Signin must use identical animation tokens
* No inline styles allowed
* Motion must not exceed 600ms
* All auth screens must be keyboard accessible

---

## 10. Personalization Contract

Content engines **must consume profile data** before rendering:

* BEGINNER → show foundations
* HARDWARE_ONLY → hide AI/ML
* FULL_ROBOTICS → unlock ROS, sensors, control loops

---

## 11. Future Extensions

* OAuth onboarding merge
* Skill reassessment checkpoints
* Adaptive difficulty scoring
* Per-module UI personalization

---

**This constitution is authoritative. Deviations require explicit revision.**

---

## 12. GitHub Actions Workflow Structure

### Directory Layout

```text
.github/
└── workflows/
    ├── auth-check.yml
    ├── ci.yml
    ├── deploy.yml
```

---

### 12.1 `ci.yml` — Build & Type Safety

**Purpose**: Ensure the project builds and types are valid before merge.

```yaml
name: CI

on:
  pull_request:
  push:
    branches: [main]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-node@v4
        with:
          node-version: 20

      - run: npm ci
      - run: npm run lint
      - run: npm run build
```

---

### 12.2 `auth-check.yml` — Auth & Constitution Guard

**Purpose**: Enforce authentication and onboarding rules defined in `sp.constitution`.

```yaml
name: Auth Constitution Check

on:
  pull_request:

jobs:
  auth-rules:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Verify constitution exists
        run: |
          test -f sp.constitution || exit 1

      - name: Verify auth directories
        run: |
          test -d app/(auth) || exit 1
          test -d app/onboarding || exit 1
```

---

### 12.3 `deploy.yml` — Production Deployment

**Purpose**: Deploy only if CI and auth rules pass.

```yaml
name: Deploy

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-node@v4
        with:
          node-version: 20

      - run: npm ci
      - run: npm run build

      - name: Deploy
        run: npm run deploy
```

---

## 13. Workflow Enforcement Rules

* `ci.yml` must pass before merge
* `auth-check.yml` must pass before deploy
* No direct commits to `main` without workflows
* Workflow changes require review

---

## 14. Future Workflow Extensions

* E2E auth tests (Playwright)
* Profile-based content snapshot tests
* Environment-specific auth secrets

---

## Response snapshot

Successfully updated the project constitution to include comprehensive authentication and onboarding specifications. The updated constitution now includes:

1. **Authentication Stack Governance**: Requirements for Next.js (App Router) + TypeScript with Better Auth, cookie-based sessions, and separation of identity from personalization data.

2. **Signup & Signin Flow Requirements**: Detailed specifications for both signup and signin flows with proper redirects and onboarding integration.

3. **User Background Profiling**: Mandatory collection of software background (skill level, known languages), hardware background (electronics experience, boards used), and learning intent (SOFTWARE_ONLY, HARDWARE_ONLY, FULL_ROBOTICS).

4. **Better Auth Configuration Standards**: Requirements for email/password authentication with cookie-based sessions.

5. **Animated UI Constitution**: Specific design principles for form containers, input fields, buttons, and loading states with CSS animations.

6. **Enforcement Rules**: Requirements for consistent animation tokens, no inline styles, motion limits, and keyboard accessibility.

7. **Personalization Contract**: Rules for content engines to consume profile data for personalized content delivery.

8. **GitHub Actions Workflow Requirements**: Specifications for CI, auth-check, and deployment workflows.

The constitution version has been updated from 1.1.0 to 1.2.0, and all existing content has been preserved while adding the new authentication and onboarding specifications. The update follows the proper formatting and maintains all existing principles while adding the new authentication-focused sections.

## Outcome

- ✅ Impact: Enhanced constitution with comprehensive auth/onboarding specifications
- 🧪 Tests: null
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: Templates may need updates to reflect new auth principles
- 🧠 Reflection: Successfully integrated new auth requirements while preserving existing governance

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All auth/onboarding specs properly integrated
- Prompt variant (if applicable): Constitution update with auth specifications
- Next experiment (smallest change to try): Update templates to reflect new auth principles
