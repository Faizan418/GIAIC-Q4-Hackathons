---
id: 001
title: Auth Onboarding Spec
stage: spec
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-auth-onboarding
branch: 001-auth-onboarding
user: user
command: /sp.specify
labels: [specification, authentication, onboarding, personalization, better-auth]
links:
  spec: ../../specs/001-auth-onboarding/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - ../../specs/001-auth-onboarding/spec.md
 - ../../specs/001-auth-onboarding/checklists/requirements.md
tests:
 - null
---

## Prompt

# sp.specify — Implementation Breakdown
---

### Scope

Defines **Signup, Signin, Onboarding, Personalization, and UI animation rules** using Better Auth.

---

### 1. Authentication

* Provider: Better Auth
* Method: Email + Password
* Session: Cookie-based

```ts
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  emailAndPassword: { enabled: true },
  session: { cookieCache: true },
});
```

---

### 2. Signup Flow

1. User submits email/password
2. Better Auth creates identity
3. Redirect → `/onboarding`

---

### 3. Signin Flow

1. User authenticates
2. Session restored
3. Profile check
4. Missing profile → onboarding

---

### 4. Onboarding Questions

#### Software Background

* Level: BEGINNER | INTERMEDIATE | ADVANCED
* Languages: Python, JS, C++, Other

#### Hardware Background

* Level: NONE | BASIC | INTERMEDIATE | ADVANCED
* Boards: Arduino, ESP32, Raspberry Pi

#### Learning Track

* SOFTWARE_ONLY
* HARDWARE_ONLY
* FULL_ROBOTICS

---

### 5. Personalization Contract

* BEGINNER → foundations first
* HARDWARE_ONLY → hide AI/ML
* FULL_ROBOTICS → unlock ROS, control systems

---

### 6. Auth UI Animation CSS

#### Card Entrance

```css
.auth-card {
  animation: cardEnter 600ms ease-out both;
}
```

#### Button Motion

```css
.auth-button:hover { transform: translateY(-2px); }
.auth-button:active { transform: scale(0.96); }
```

---

### 7. Enforcement

* Auth screens must reuse shared CSS
* No inline animation styles

---

## File 2: `sp.specify.workflow.md`

### Scope

Defines **CI/CD workflow structure and enforcement**.

---

### 1. Directory Structure

```text
.github/
└── workflows/
    ├── ci.yml
    ├── auth-check.yml
    └── deploy.yml
```

---

### 2. CI Workflow (`ci.yml`)

* Install dependencies
* Lint
* Build

---

### 3. Auth Guard Workflow (`auth-check.yml`)

* Verify `sp.constitution` exists
* Verify auth/onboarding directories

---

### 4. Deployment Workflow (`deploy.yml`)

* Runs only on `main`
* Requires CI pass

---

### 5. Workflow Rules

* No direct deploy without checks
* Workflow changes require review

---

## Ownership & Change Control

* Each file versioned independently
* Breaking changes require explicit approval

---

**This split is mandatory for scale and governance.**

## Response snapshot

Successfully created comprehensive specification for the Auth & Onboarding System with Better Auth. The specification includes:

1. **User Scenarios & Testing**: Four prioritized user stories covering registration/authentication (P1), onboarding/profile collection (P2), personalized content delivery (P3), and animated UI experience (P2).

2. **Functional Requirements**: 16 specific, testable requirements covering authentication, profile collection, personalization, UI animations, and error handling.

3. **Key Entities**: Identified UserProfile, AuthSession, and PersonalizationRule as core data entities.

4. **Success Criteria**: 8 measurable outcomes with specific metrics for user experience, performance, and business value.

5. **Quality Validation**: Created requirements checklist confirming all items pass validation.

The specification follows the template structure and focuses on user value without implementation details, meeting the requirements for a technology-agnostic specification that can guide planning and development.

## Outcome

- ✅ Impact: Complete auth/onboarding specification ready for planning
- 🧪 Tests: null
- 📁 Files: 2 files created (spec.md, requirements.md)
- 🔁 Next prompts: Ready for /sp.plan
- 🧠 Reflection: Successfully translated user requirements into comprehensive specification

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All specification requirements met
- Prompt variant (if applicable): Specification for auth/onboarding feature
- Next experiment (smallest change to try): Proceed to planning phase
