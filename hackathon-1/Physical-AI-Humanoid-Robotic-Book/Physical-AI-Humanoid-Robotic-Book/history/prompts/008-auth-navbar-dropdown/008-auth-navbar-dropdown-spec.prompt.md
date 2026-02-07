---
id: 008
title: Auth Navbar Dropdown Spec
stage: spec
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: auth-navbar-dropdown
branch: 007-user-auth-better-auth
user: Claude
command: /sp.specify
labels: ["auth", "navbar", "dropdown", "docusaurus", "spec"]
links:
  spec: ../specs/008-auth-navbar-dropdown/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - ../specs/008-auth-navbar-dropdown/spec.md
 - ../specs/008-auth-navbar-dropdown/checklists/requirements.md
tests:
 -

---

## Prompt

# sp.specify — Auth-Aware Navbar Dropdown (Docusaurus + Better Auth)

## Purpose

Define strict requirements for implementing an **authentication-aware navbar dropdown** in a **Docusaurus-based book platform**, integrated with **Better Auth**, ensuring clickability, CSS reliability, accessibility, and long-term compatibility with Docusaurus theme updates.

---

## Scope

This specification applies to:

* Auth-aware navbar items (Sign In / Sign Up / Dashboard / Logout)
* Session-based UI switching
* Dropdown rendering and interaction behavior
* Integration boundaries between Docusaurus Navbar and Better Auth

Out of scope:

* Backend authentication provider setup
* Database, user models, or authorization rules

---

## Architectural Principles

1. **Single Navbar Ownership**

   * The default Docusaurus navbar MUST NOT be wrapped, duplicated, or re-rendered.
   * Authentication UI MUST be injected using the official **custom NavbarItem extension mechanism**.

2. **Framework Ownership of Behavior**

   * Dropdown open/close behavior MUST be handled by Docusaurus.
   * Manual document-level click listeners MUST NOT be implemented.

3. **Separation of Concerns**

   * Navbar configuration handles placement only.
   * Authentication logic executes inside React components.

---

## Required Integration Pattern

### Navbar Configuration

* A custom navbar item MUST be declared using `type`.

```js
{
  type: 'authDropdown',
  position: 'right',
}
```

---

### Custom Navbar Item Location

* Component MUST exist at:

```
src/theme/NavbarItem/AuthDropdown.tsx
```

* Component name MUST match the `type` value (case-insensitive mapping).

---

## Authentication State Handling

1. Session data MUST be accessed via a dedicated session context (e.g., `useSession`).
2. The component MUST support three states:

   * `loading`
   * `authenticated`
   * `unauthenticated`
3. While `loading === true`, the navbar item MUST NOT render.

---

## Dropdown Behavior Requirements

1. Dropdown MUST use native Docusaurus classes:

   * `navbar__item`
   * `dropdown`
   * `dropdown--hoverable`

2. The dropdown toggle MUST be an anchor (`<a>`) or button styled as a link using:

   * `navbar__link`

3. Dropdown menu MUST be structured as:

```html
<ul class="dropdown__menu">
  <li>
    <a class="dropdown__link"></a>
  </li>
</ul>
```

4. `<ul>` elements MUST NOT contain `<a>` or `<button>` directly without `<li>` wrappers.

---

## Auth-Aware Menu Rules

### Unauthenticated State

* Navbar label MUST display: `Sign In`
* Dropdown MUST contain:

  * Sign In → `/auth/signin`
  * Sign Up → `/auth/signup`

### Authenticated State

* Navbar label MUST display: `Dashboard`
* Dropdown MUST contain:

  * Dashboard → `/dashboard`
  * Logout (button)

---

## Logout Behavior

1. Logout MUST be triggered from a `<button>` element.
2. Logout MUST call Better Auth `signOut()`.
3. After logout, the user MUST be redirected to `/`.
4. Logout buttons MUST use class `dropdown__link` for visual consistency.

---

## Styling Requirements

1. All navbar styling MUST rely on existing Docusaurus classes.
2. Custom CSS MAY be added but MUST NOT override core dropdown behavior.
3. No inline styles are permitted in navbar items.

---

## Explicit Prohibitions

* ❌ No wrapping or re-exporting `@theme-original/Navbar`
* ❌ No `document.addEventListener` for dropdown handling
* ❌ No auth logic inside `docusaurus.config.js`
* ❌ No invalid HTML structures inside dropdowns

---

## Accessibility Requirements

1. Markup MUST be semantic and valid.
2. Dropdown MUST support keyboard navigation via Docusaurus defaults.
3. Buttons MUST include `type="button"`.

---

## Acceptance Criteria

Implementation is compliant ONLY IF:

1. Navbar renders a single auth-aware dropdown item.
2. Dropdown opens and closes correctly on desktop and mobile.
3. All links are clickable and styled correctly.
4. Logout works reliably without console errors.
5. No custom event listeners exist for dropdown control.
6. The solution survives Docusaurus minor and patch upgrades.

---

## Quality Gate

Any implementation that bypasses the official NavbarItem extension mechanism SHALL be considered **non-compliant** with this specification.

---

## Notes

This specification enforces Docusaurus-native patterns to avoid UI breakage, CSS inconsistency, and long-term maintenance risks in a documentation-first product.

## Response snapshot

# Feature Specification: Auth-Aware Navbar Dropdown

**Feature Branch**: `008-auth-navbar-dropdown`
**Created**: 2025-12-19
**Status**: Draft
**Input**: User description: "# sp.specify — Auth-Aware Navbar Dropdown (Docusaurus + Better Auth)

## Purpose

Define strict requirements for implementing an **authentication-aware navbar dropdown** in a **Docusaurus-based book platform**, integrated with **Better Auth**, ensuring clickability, CSS reliability, accessibility, and long-term compatibility with Docusaurus theme updates.

---

## Scope

This specification applies to:

* Auth-aware navbar items (Sign In / Sign Up / Dashboard / Logout)
* Session-based UI switching
* Dropdown rendering and interaction behavior
* Integration boundaries between Docusaurus Navbar and Better Auth

Out of scope:

* Backend authentication provider setup
* Database, user models, or authorization rules

---

## Architectural Principles

1. **Single Navbar Ownership**

   * The default Docusaurus navbar MUST NOT be wrapped, duplicated, or re-rendered.
   * Authentication UI MUST be injected using the official **custom NavbarItem extension mechanism**.

2. **Framework Ownership of Behavior**

   * Dropdown open/close behavior MUST be handled by Docusaurus.
   * Manual document-level click listeners MUST NOT be implemented.

3. **Separation of Concerns**

   * Navbar configuration handles placement only.
   * Authentication logic executes inside React components.

---

## Required Integration Pattern

### Navbar Configuration

* A custom navbar item MUST be declared using `type`.

```js
{
  type: 'authDropdown',
  position: 'right',
}
```

---

### Custom Navbar Item Location

* Component MUST exist at:

```
src/theme/NavbarItem/AuthDropdown.tsx
```

* Component name MUST match the `type` value (case-insensitive mapping).

---

## Authentication State Handling

1. Session data MUST be accessed via a dedicated session context (e.g., `useSession`).
2. The component MUST support three states:

   * `loading`
   * `authenticated`
   * `unauthenticated`
3. While `loading === true`, the navbar item MUST NOT render.

---

## Dropdown Behavior Requirements

1. Dropdown MUST use native Docusaurus classes:

   * `navbar__item`
   * `dropdown`
   * `dropdown--hoverable`

2. The dropdown toggle MUST be an anchor (`<a>`) or button styled as a link using:

   * `navbar__link`

3. Dropdown menu MUST be structured as:

```html
<ul class="dropdown__menu">
  <li>
    <a class="dropdown__link"></a>
  </li>
</ul>
```

4. `<ul>` elements MUST NOT contain `<a>` or `<button>` directly without `<li>` wrappers.

---

## Auth-Aware Menu Rules

### Unauthenticated State

* Navbar label MUST display: `Sign In`
* Dropdown MUST contain:

  * Sign In → `/auth/signin`
  * Sign Up → `/auth/signup`

### Authenticated State

* Navbar label MUST display: `Dashboard`
* Dropdown MUST contain:

  * Dashboard → `/dashboard`
  * Logout (button)

---

## Logout Behavior

1. Logout MUST be triggered from a `<button>` element.
2. Logout MUST call Better Auth `signOut()`.
3. After logout, the user MUST be redirected to `/`.
4. Logout buttons MUST use class `dropdown__link` for visual consistency.

---

## Styling Requirements

1. All navbar styling MUST rely on existing Docusaurus classes.
2. Custom CSS MAY be added but MUST NOT override core dropdown behavior.
3. No inline styles are permitted in navbar items.

---

## Explicit Prohibitions

* ❌ No wrapping or re-exporting `@theme-original/Navbar`
* ❌ No `document.addEventListener` for dropdown handling
* ❌ No auth logic inside `docusaurus.config.js`
* ❌ No invalid HTML structures inside dropdowns

---

## Accessibility Requirements

1. Markup MUST be semantic and valid.
2. Dropdown MUST support keyboard navigation via Docusaurus defaults.
3. Buttons MUST include `type="button"`.

---

## Acceptance Criteria

Implementation is compliant ONLY IF:

1. Navbar renders a single auth-aware dropdown item.
2. Dropdown opens and closes correctly on desktop and mobile.
3. All links are clickable and styled correctly.
4. Logout works reliably without console errors.
5. No custom event listeners exist for dropdown control.
6. The solution survives Docusaurus minor and patch upgrades.

---

## Quality Gate

Any implementation that bypasses the official NavbarItem extension mechanism SHALL be considered **non-compliant** with this specification.

---

## Notes

This specification enforces Docusaurus-native patterns to avoid UI breakage, CSS inconsistency, and long-term maintenance risks in a documentation-first product."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Unauthenticated User Access (Priority: P1)

When an unauthenticated user visits the book platform, they should see a "Sign In" link in the navbar that opens a dropdown with Sign In and Sign Up options. This allows them to easily access authentication flows.

**Why this priority**: This is the foundational experience for new users and must work reliably to enable onboarding.

**Independent Test**: Can be fully tested by visiting the site as an unauthenticated user and verifying the dropdown shows "Sign In" with working links to authentication pages.

**Acceptance Scenarios**:

1. **Given** user is not logged in, **When** user visits the site, **Then** navbar shows "Sign In" link
2. **Given** user sees "Sign In" link, **When** user hovers/clicks the link, **Then** dropdown opens with Sign In and Sign Up options
3. **Given** dropdown is open with auth options, **When** user clicks Sign In/Sign Up, **Then** user is navigated to appropriate auth page

---

### User Story 2 - Authenticated User Access (Priority: P1)

When an authenticated user visits the book platform, they should see a "Dashboard" link in the navbar that opens a dropdown with Dashboard and Logout options. This provides quick access to their account features.

**Why this priority**: This is the core experience for returning users and must work reliably to maintain engagement.

**Independent Test**: Can be fully tested by logging in and verifying the dropdown shows "Dashboard" with working links to dashboard and logout functionality.

**Acceptance Scenarios**:

1. **Given** user is logged in, **When** user visits the site, **Then** navbar shows "Dashboard" link
2. **Given** user sees "Dashboard" link, **When** user hovers/clicks the link, **Then** dropdown opens with Dashboard and Logout options
3. **Given** dropdown is open with dashboard options, **When** user clicks Dashboard, **Then** user is navigated to dashboard page
4. **Given** dropdown is open with logout option, **When** user clicks Logout, **Then** user is logged out and redirected to homepage

---

### User Story 3 - Loading State Handling (Priority: P2)

When the authentication status is still loading, the navbar item should not render to avoid flickering or incorrect state display.

**Why this priority**: This ensures a smooth user experience during the authentication check process.

**Independent Test**: Can be tested by simulating slow authentication loading and verifying the navbar item remains hidden until status is determined.

**Acceptance Scenarios**:

1. **Given** authentication status is loading, **When** page renders, **Then** no auth navbar item is visible
2. **Given** authentication status loads to authenticated, **When** status resolves, **Then** "Dashboard" link appears
3. **Given** authentication status loads to unauthenticated, **When** status resolves, **Then** "Sign In" link appears

---

### Edge Cases

- What happens when authentication API fails? The system should default to unauthenticated state and show "Sign In" link.
- How does the system handle expired sessions? It should detect this and show unauthenticated state.
- What if the Better Auth library is not initialized properly? The system should gracefully handle the error and default to unauthenticated state.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST detect user authentication status using Better Auth session context
- **FR-002**: System MUST render "Sign In" link when user is unauthenticated
- **FR-003**: System MUST render "Dashboard" link when user is authenticated
- **FR-004**: System MUST display Sign In and Sign Up links in dropdown when unauthenticated
- **FR-005**: System MUST display Dashboard and Logout links in dropdown when authenticated
- **FR-006**: System MUST use Docusaurus native navbar classes for proper styling
- **FR-007**: System MUST handle loading state by not rendering the navbar item
- **FR-008**: System MUST implement logout functionality that calls Better Auth `signOut()`
- **FR-009**: System MUST redirect user to homepage after logout
- **FR-010**: System MUST ensure all dropdown elements are properly structured with correct HTML semantics
- **FR-011**: System MUST use Docusaurus dropdown behavior (no custom event listeners)
- **FR-012**: System MUST ensure accessibility compliance with keyboard navigation

### Key Entities *(include if feature involves data)*

- **Authentication State**: Represents the current authentication status (loading, authenticated, unauthenticated)
- **Navbar Item**: Represents the custom authentication-aware dropdown component
- **Better Auth Session**: Represents the user session data from Better Auth

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can access authentication flows (Sign In/Sign Up) within 1 click from any page
- **SC-002**: Users can access dashboard and logout functionality within 1 click when authenticated
- **SC-003**: Authentication state detection completes within 500ms of page load
- **SC-004**: 100% of users can successfully log out and be redirected to homepage
- **SC-005**: All dropdown functionality works across desktop and mobile devices
- **SC-006**: Implementation survives Docusaurus minor and patch upgrades without breaking
- **SC-007**: No console errors occur during authentication state transitions

## Outcome

- ✅ Impact: Created comprehensive specification for auth-aware navbar dropdown feature with clear requirements and acceptance criteria
- 🧪 Tests: No tests run yet, specification created for future implementation
- 📁 Files: Created spec.md and requirements checklist in new feature directory
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification follows Docusaurus best practices and Better Auth integration patterns
- 📝 Created checklist file to validate specification quality

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable at spec stage
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Proceed to planning phase
