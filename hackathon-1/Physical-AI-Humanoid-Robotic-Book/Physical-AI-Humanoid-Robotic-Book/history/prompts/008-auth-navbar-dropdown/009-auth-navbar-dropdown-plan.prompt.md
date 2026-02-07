---
id: 009
title: Auth Navbar Dropdown Plan
stage: plan
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: auth-navbar-dropdown
branch: 007-user-auth-better-auth
user: Claude
command: /sp.plan
labels: ["auth", "navbar", "dropdown", "docusaurus", "plan"]
links:
  spec: ../specs/008-auth-navbar-dropdown/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - ../specs/008-auth-navbar-dropdown/plan.md
 - ../specs/008-auth-navbar-dropdown/research.md
 - ../specs/008-auth-navbar-dropdown/data-model.md
 - ../specs/008-auth-navbar-dropdown/quickstart.md
 - ../specs/008-auth-navbar-dropdown/contracts/component-contract.md
 - ../specs/008-auth-navbar-dropdown/contracts/
tests:
 -

---

## Prompt

# sp.plan — Auth-Aware Navbar Implementation (Docusaurus + Better Auth)

## Objective

Implement an authentication-aware navbar dropdown in a Docusaurus-based book site that fully complies with **sp.specify — Auth-Aware Navbar Dropdown**, ensuring functional Sign In / Sign Up / Dashboard / Logout flows using **Better Auth**.

---

## Phase 1 — Preparation & Cleanup

### 1.1 Remove Non-Compliant Code

* Delete any overrides of `@theme-original/Navbar`.
* Remove manual dropdown state management (`useState`, `document.addEventListener`).
* Remove invalid dropdown markup (`<ul>` containing non-`<li>` children).

**Exit Criteria:**

* Only the default Docusaurus navbar remains active.

---

## Phase 2 — Navbar Extension Setup

### 2.1 Declare Custom Navbar Item

Update `docusaurus.config.js`:

```js
navbar: {
  items: [
    {
      type: 'authDropdown',
      position: 'right',
    },
  ],
},
```

**Exit Criteria:**

* Navbar renders without errors.

---

### 2.2 Create Custom Navbar Item Component

**File:**

```
src/theme/NavbarItem/AuthDropdown.tsx
```

**Responsibilities:**

* Read authentication state via session context.
* Render correct label based on auth state.
* Defer dropdown behavior to Docusaurus.

**Exit Criteria:**

* Component mounts successfully.

---

## Phase 3 — Authentication State Integration

### 3.1 Session Context Integration

* Use existing `useSession` hook.
* Handle three states:

  * `loading`
  * `authenticated`
  * `unauthenticated`

**Rules:**

* Render nothing during loading.

**Exit Criteria:**

* Navbar item appears only after session resolution.

---

## Phase 4 — Dropdown Content Implementation

### 4.1 Unauthenticated State

**Navbar Label:**

* `Sign In`

**Dropdown Items:**

* Sign In → `/auth/signin`
* Sign Up → `/auth/signup`

---

### 4.2 Authenticated State

**Navbar Label:**

* `Dashboard`

**Dropdown Items:**

* Dashboard → `/dashboard`
* Logout → button

---

## Phase 5 — Logout Flow

### 5.1 Logout Action

* Invoke `signOut()` from Better Auth.
* Redirect user to `/` after success.

**Exit Criteria:**

* Session clears correctly.
* Navbar switches to unauthenticated state.

---

## Phase 6 — Styling & UX Validation

### 6.1 Styling Rules

* Use Docusaurus default classes only:

  * `navbar__item`
  * `navbar__link`
  * `dropdown__menu`
  * `dropdown__link`

* No inline styles.

* No JS-based dropdown control.

---

## Phase 7 — Accessibility & Responsiveness

### 7.1 Accessibility Checks

* Valid semantic HTML (`ul > li > a/button`).
* Keyboard navigation supported by default.
* Buttons use `type="button"`.

### 7.2 Mobile Validation

* Dropdown works in mobile navbar.
* No z-index or overlay conflicts.

---

## Phase 8 — Verification & Acceptance

### 8.1 Functional Tests

* Sign In link navigates correctly.
* Sign Up link navigates correctly.
* Dashboard link accessible when authenticated.
* Logout clears session and redirects.

### 8.2 Non-Regression Checks

* No console warnings or errors.
* Navbar renders exactly once.
* Compatible with Docusaurus theme updates.

---

## Deliverables

* `AuthDropdown.tsx` custom navbar item
* Updated `docusaurus.config.js`
* Clean, compliant navbar behavior

---

## Completion Criteria

The plan is complete when:

* Implementation satisfies all **sp.specify** requirements
* Navbar authentication UI is stable, styled, and functional
* No framework constraints are violated

---

## Notes

This plan intentionally prioritizes framework-native behavior over custom JS to ensure long-term maintainability in a documentation-first product.

## Response snapshot

# Implementation Plan: Auth-Aware Navbar Dropdown

**Branch**: `008-auth-navbar-dropdown` | **Date**: 2025-12-19 | **Spec**: [../specs/008-auth-navbar-dropdown/spec.md](../specs/008-auth-navbar-dropdown/spec.md)
**Input**: Feature specification from `/specs/008-auth-navbar-dropdown/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an authentication-aware navbar dropdown in Docusaurus using Better Auth integration. The solution will create a custom NavbarItem component that displays different content based on authentication state: "Sign In" with dropdown options (Sign In, Sign Up) for unauthenticated users, and "Dashboard" with dropdown options (Dashboard, Logout) for authenticated users. The implementation will follow Docusaurus-native patterns without custom dropdown state management.

## Technical Context

**Language/Version**: TypeScript 5.0+ (for Docusaurus compatibility), React 18+
**Primary Dependencies**: Docusaurus v3, Better Auth, React
**Storage**: N/A (frontend only, data handled by backend services)
**Testing**: Jest, React Testing Library
**Target Platform**: Web (Docusaurus documentation site)
**Project Type**: Web (Docusaurus-based documentation site with authentication)
**Performance Goals**: <100ms authentication state detection, <200ms dropdown rendering
**Constraints**: Must use Docusaurus native dropdown behavior, no custom event listeners, comply with accessibility standards
**Scale/Scope**: Single-page application with responsive design for desktop and mobile

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution file:
- ✅ Must use Docusaurus (TypeScript) for book authoring - COMPLIANT: Using Docusaurus NavbarItem extension
- ✅ Authentication must use Better Auth - COMPLIANT: Implementation uses Better Auth session context
- ✅ Follow accessibility standards - COMPLIANT: Plan includes accessibility checks
- ✅ Responsive design principles - COMPLIANT: Mobile validation included in plan
- ✅ No inline styles allowed - COMPLIANT: Plan specifies use of Docusaurus classes only

## Project Structure

### Documentation (this feature)

```text
specs/008-auth-navbar-dropdown/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── theme/
│   └── NavbarItem/
│       └── AuthDropdown.tsx    # Custom authentication-aware navbar item
└── components/
    └── [other components]

docusaurus.config.ts    # Configuration with navbar items
```

**Structure Decision**: Single project with Docusaurus theme customization. The AuthDropdown component will be placed in the Docusaurus theme override directory following Docusaurus conventions for custom NavbarItem components.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |

---

# Research: Auth-Aware Navbar Dropdown Implementation

## Decision: Use Docusaurus NavbarItem Extension Mechanism
**Rationale**: The specification requires using the official Docusaurus NavbarItem extension mechanism to ensure compatibility with Docusaurus dropdown behavior and avoid custom event listeners. This approach follows Docusaurus best practices and ensures long-term maintainability.

**Alternatives considered**:
- Direct navbar override: Would break Docusaurus native behavior
- Custom dropdown implementation: Would violate specification requirements
- Third-party dropdown library: Would add unnecessary complexity

## Decision: Better Auth Session Context Integration
**Rationale**: Using Better Auth's session context (`useSession`) provides a clean separation between authentication logic and UI rendering. This follows the specification requirement to access session data via dedicated session context.

**Alternatives considered**:
- Manual API calls to check auth status: Would be less efficient
- Global state management: Would add unnecessary complexity

## Decision: Docusaurus Native CSS Classes
**Rationale**: Using Docusaurus native classes (`navbar__item`, `dropdown`, `dropdown--hoverable`, `navbar__link`, `dropdown__menu`, `dropdown__link`) ensures consistent styling with the rest of the navbar and maintains compatibility with theme updates.

**Alternatives considered**:
- Custom CSS classes: Would risk breaking with theme updates
- Inline styles: Prohibited by specification

## Decision: Loading State Handling
**Rationale**: The component will not render during loading state to prevent flickering and ensure a smooth user experience, as specified in the requirements.

**Alternatives considered**:
- Show placeholder during loading: Could cause layout shifts
- Show default state during loading: Could display incorrect information

## Decision: Logout Implementation
**Rationale**: Using Better Auth's `signOut()` function with redirect to homepage ensures proper session cleanup and follows the specification requirements for logout behavior.

**Alternatives considered**:
- Manual session clearing: Less secure and reliable
- Custom redirect logic: More complex than necessary

---

# Data Model: Auth-Aware Navbar Dropdown

## Entities

### Authentication State
**Description**: Represents the current authentication status of the user
- **States**: `loading`, `authenticated`, `unauthenticated`
- **Source**: Better Auth session context
- **Validation**: Must be one of the three defined states

### Navbar Item
**Description**: Represents the custom authentication-aware dropdown component
- **Properties**:
  - `type`: 'authDropdown' (for Docusaurus navbar configuration)
  - `position`: 'right' (navbar placement)
- **Validation**: Type must match component file name for Docusaurus resolution

### Better Auth Session
**Description**: Represents the user session data from Better Auth
- **Properties**:
  - `user`: User object when authenticated (null when unauthenticated)
  - `isLoading`: Boolean indicating session loading state
  - `isAuthenticated`: Boolean indicating authentication status
- **Validation**: Follows Better Auth API contract

## State Transitions

### Authentication State Transitions
- `loading` → `authenticated`: When session loads and user is logged in
- `loading` → `unauthenticated`: When session loads and user is not logged in
- `authenticated` → `unauthenticated`: When user logs out
- `unauthenticated` → `authenticated`: When user logs in

### Navbar Label Transitions
- When state is `loading`: Component does not render
- When state is `unauthenticated`: Label shows "Sign In"
- When state is `authenticated`: Label shows "Dashboard"

---

# Quickstart: Auth-Aware Navbar Dropdown

## Overview
This guide will help you implement an authentication-aware navbar dropdown that integrates with Better Auth and follows Docusaurus best practices.

## Prerequisites
- Docusaurus v3 installed
- Better Auth configured in your project
- TypeScript 5.0+ environment

## Implementation Steps

### 1. Configure Navbar Item
Update your `docusaurus.config.ts` to include the auth dropdown:

```js
module.exports = {
  // ... other config
  navbar: {
    items: [
      // ... other items
      {
        type: 'authDropdown',
        position: 'right',
      },
    ],
  },
};
```

### 2. Create AuthDropdown Component
Create the component at `src/theme/NavbarItem/AuthDropdown.tsx`:

```tsx
import React from 'react';
import { useSession, signOut } from 'better-auth/react';
import { useHistory } from '@docusaurus/router';

const AuthDropdown = () => {
  const { data: session, isLoading } = useSession();

  if (isLoading) {
    return null; // Don't render during loading
  }

  if (session?.user) {
    // Authenticated state
    return (
      <div className="navbar__item dropdown dropdown--hoverable">
        <a className="navbar__link" href="/dashboard">Dashboard</a>
        <ul className="dropdown__menu">
          <li><a className="dropdown__link" href="/dashboard">Dashboard</a></li>
          <li>
            <button
              type="button"
              className="dropdown__link"
              onClick={async () => {
                await signOut();
                window.location.href = '/';
              }}
            >
              Logout
            </button>
          </li>
        </ul>
      </div>
    );
  } else {
    // Unauthenticated state
    return (
      <div className="navbar__item dropdown dropdown--hoverable">
        <a className="navbar__link">Sign In</a>
        <ul className="dropdown__menu">
          <li><a className="dropdown__link" href="/auth/signin">Sign In</a></li>
          <li><a className="dropdown__link" href="/auth/signup">Sign Up</a></li>
        </ul>
      </div>
    );
  }
};

export default AuthDropdown;
```

### 3. Verify Implementation
- Ensure the navbar renders correctly in both authenticated and unauthenticated states
- Test that dropdown opens/closes properly using Docusaurus native behavior
- Verify logout functionality works correctly
- Check that loading state is handled properly

## Key Features
- Uses Docusaurus native dropdown behavior
- Integrates with Better Auth session context
- Follows accessibility standards
- Responsive design for mobile and desktop
- Proper loading state handling

---

# Component Contract: AuthDropdown

## Overview
The AuthDropdown component provides an authentication-aware navbar dropdown that displays different content based on user authentication status.

## Component Interface

### Props
The component does not accept any props as it's a Docusaurus NavbarItem extension that gets configured via the navbar configuration.

### Dependencies
- `better-auth/react`: For session management and authentication functions
- `@docusaurus/router`: For navigation after logout (if needed)

## Behavior Contracts

### Authentication State Handling
- **Loading State**: When `isLoading` is true, component returns `null` (does not render)
- **Authenticated State**: Shows "Dashboard" as the dropdown label with Dashboard and Logout options
- **Unauthenticated State**: Shows "Sign In" as the dropdown label with Sign In and Sign Up options

### Session Integration
- **Input**: Uses `useSession()` hook from Better Auth to get current session state
- **Output**: Renders appropriate UI based on session data
- **Error Handling**: Defaults to unauthenticated state if session check fails

### Navigation Contracts
- **Sign In**: Navigates to `/auth/signin`
- **Sign Up**: Navigates to `/auth/signup`
- **Dashboard**: Navigates to `/dashboard`
- **Logout**: Calls `signOut()` and redirects to `/`

### Styling Contracts
- **Classes Used**:
  - `navbar__item`: Container for navbar item
  - `dropdown`: Dropdown container
  - `dropdown--hoverable`: Enables hover behavior
  - `navbar__link`: Link styling
  - `dropdown__menu`: Dropdown menu container
  - `dropdown__link`: Dropdown item styling

### Accessibility Contracts
- **Keyboard Navigation**: Supports Docusaurus default keyboard navigation
- **Button Attributes**: Logout button includes `type="button"` attribute
- **Semantic HTML**: Uses proper HTML structure (`ul > li > a/button`)

## State Transitions
- Component automatically transitions between states based on session data
- No external state management required
- Follows Docusaurus dropdown behavior for open/close

## Error Handling
- If authentication API fails, defaults to unauthenticated state
- If Better Auth is not initialized, defaults to unauthenticated state
- Logout errors are handled by Better Auth library

## Outcome

- ✅ Impact: Created comprehensive implementation plan for auth-aware navbar dropdown with research, data model, quickstart guide, and contracts
- 🧪 Tests: No tests run yet, planning artifacts created for future implementation
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, and contracts directory
- 🔁 Next prompts: Ready for task generation phase with /sp.tasks
- 🧠 Reflection: Planning followed Docusaurus best practices and Better Auth integration patterns
- 📝 Created research, data model, quickstart, and contracts documentation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable at planning stage
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Proceed to task generation phase
