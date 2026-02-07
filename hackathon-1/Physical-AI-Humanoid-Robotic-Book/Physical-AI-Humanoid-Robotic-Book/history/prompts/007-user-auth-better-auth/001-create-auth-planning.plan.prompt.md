---
id: 001
title: Create auth planning
stage: plan
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5-20250929
feature: 007-user-auth-better-auth
branch: 007-user-auth-better-auth
user: Claude
command: /sp.plan
labels: [auth, planning, better-auth, docusaurus]
links:
  spec: ../specs/007-user-auth-better-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - ../specs/007-user-auth-better-auth/plan.md
 - ../specs/007-user-auth-better-auth/research.md
 - ../specs/007-user-auth-better-auth/data-model.md
 - ../specs/007-user-auth-better-auth/quickstart.md
 - ../specs/007-user-auth-better-auth/contracts/auth-api.yaml
tests:
 - null
---

## Prompt

# sp.plan — Authentication, Authorization & Personalization

## Objective

Provide a **step-by-step, execution-ready implementation plan** for delivering authentication, authorization, and personalization features defined in `sp.specify`, using **Docusaurus + Better Auth**.

This plan is ordered, dependency-aware, and suitable for direct developer execution.

---

## Phase 0 — Preconditions

### 0.1 Repository Readiness

* Docusaurus project MUST build successfully
* Node.js LTS installed
* Git repository clean

### 0.2 Dependency Installation

```bash
npm install better-auth
```

Acceptance:

* `better-auth` present in `package.json`
* No build errors after install

---

## Phase 1 — Auth Infrastructure Setup

### 1.1 Create Auth API Routes

Create directory:

```
/api/auth/
```

Files to implement:

* `signin.ts`
* `signup.ts`
* `logout.ts`

Actions:

* Wire Better Auth `signIn`, `signUp`, `signOut`
* Enable cookie-based sessions

Acceptance:

* API routes respond with 200
* Session cookie set on sign in

---

## Phase 2 — Frontend Auth Pages

### 2.1 Sign In Page

Path:

```
src/pages/auth/signin.tsx
```

Actions:

* Add Sign In UI
* Call `/api/auth/signin`
* Redirect to `/dashboard`

Acceptance:

* User can sign in
* Session persists on refresh

---

### 2.2 Sign Up Page

Path:

```
src/pages/auth/signup.tsx
```

Actions:

* Collect software/hardware background
* POST data to `/api/auth/signup`
* Redirect to `/dashboard`

Acceptance:

* Background data saved
* User automatically logged in

---

## Phase 3 — Middleware & Protection

### 3.1 Auth Middleware

Actions:

* Implement middleware using `getSession`
* Redirect unauthenticated users to `/auth/signin`

Protected routes:

* `/dashboard`
* `/docs/advanced/*`

Acceptance:

* Guest access blocked
* Auth users pass through

---

## Phase 4 — Navbar Integration

### 4.1 Static Navbar

Actions:

* Add Sign In / Sign Up links to navbar

### 4.2 Auth-Based Navbar Toggle

Actions:

* Detect session on load
* Show:

  * Guest → Sign In / Sign Up
  * Auth → Dashboard / Logout

Acceptance:

* Navbar updates immediately after login/logout

---

## Phase 5 — User Dashboard

### 5.1 Dashboard Page

Path:

```
src/pages/dashboard.tsx
```

Actions:

* Fetch session data
* Display user metadata
* Show personalized welcome

Acceptance:

* Only accessible when authenticated
* Correct user data shown

---

## Phase 6 — Protected Documentation

### 6.1 Docs Segmentation

Actions:

* Mark selected docs as protected
* Apply middleware rules

Acceptance:

* Direct URL access blocked for guests

---

## Phase 7 — QA & Verification

### 7.1 Manual Tests

* Sign up new user
* Verify metadata persistence
* Logout and confirm session cleared
* Attempt protected access as guest

### 7.2 Regression

* Docs build passes
* No navbar layout regressions

---

## Phase 8 — Deployment Readiness

Actions:

* Verify env variables
* Ensure cookies work in production
* Confirm redirects on hosted domain

Acceptance:

* Auth flow works end-to-end in production

---

## Completion Criteria

* All phases executed in order
* All acceptance checks pass
* No auth logic outside Better Auth

---

## Status

**Planned — Ready for execution**

## Response snapshot

# Implementation Plan: Better Auth Integration for Docusaurus Platform

**Branch**: `007-user-auth-better-auth` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/007-user-auth-better-auth/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of authentication, authorization, and personalization features using Better Auth in the Docusaurus-based Physical AI / Humanoid Robotics platform. The solution includes user sign-up/sign-in with background data collection, dynamic navbar behavior, protected documentation pages, and personalized dashboard with cookie-based session management.

## Technical Context

**Language/Version**: TypeScript 5.0+ (for Docusaurus compatibility)
**Primary Dependencies**: Better Auth, Docusaurus v3, React 18+
**Storage**: User data and metadata stored via Better Auth with cookie-based sessions
**Testing**: Jest, React Testing Library (for UI components)
**Target Platform**: Web browser, compatible with modern browsers
**Project Type**: Web application (Docusaurus documentation site with authentication)
**Performance Goals**: <500ms navbar state updates, <2s page load times with auth middleware
**Constraints**: Cookie-based sessions only, no custom auth logic outside Better Auth, secure storage of user metadata
**Scale/Scope**: Support for 1000+ concurrent authenticated users, persistent user profiles

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Authentication Stack Compliance (Section 90-100)
- ✅ **Better Auth Required**: Using Better Auth as specified in constitution
- ✅ **Session Strategy**: Cookie-based sessions as mandated
- ✅ **TypeScript Required**: Using TypeScript 5.0+ as per constitution

### Docusaurus Integration (Section 49-50)
- ✅ **Docusaurus-First Structure**: Maintaining Docusaurus docs folder structure
- ✅ **Frontend Integration**: Auth UI integrated into Docusaurus theme

### Quality and Verification (Section 55-56)
- ✅ **Human Validation**: Required for Claude Code assistance
- ✅ **Proper Error Handling**: As specified in requirements

### Tooling Mandate (Section 58-59)
- ✅ **Mandatory Tooling**: Using Docusaurus, TypeScript as required
- ✅ **No Violations**: All tools align with constitution

## Project Structure

### Documentation (this feature)

```text
specs/007-user-auth-better-auth/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application with Docusaurus frontend
src/
├── pages/
│   ├── auth/
│   │   ├── signin.tsx
│   │   ├── signup.tsx
│   │   └── dashboard.tsx
│   └── components/
│       └── navbar/
│           └── AuthAwareNavbar.tsx
├── services/
│   └── auth/
│       └── better-auth-client.ts
├── theme/
│   └── Navbar/
│       └── index.tsx
└── types/
    └── auth.ts

api/
└── auth/
    ├── signin.ts
    ├── signup.ts
    └── logout.ts

docs/
└── advanced/            # Protected documentation pages
    ├── ...
    └── ...

static/
└── css/
    └── auth.css         # Custom authentication styles
```

**Structure Decision**: Web application structure chosen since this is a Docusaurus site with authentication features. The frontend components are in src/pages/ and the backend API routes are in api/ directory.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|

---
## Research: Better Auth Integration for Docusaurus Platform

## Decision: Better Auth Integration Strategy

**Rationale**: Better Auth was selected as the authentication provider based on the feature specification requirements. It provides a complete authentication solution with cookie-based sessions, user metadata storage, and integration capabilities suitable for Docusaurus applications.

## Decision: Docusaurus Authentication Integration Pattern

**Rationale**: For Docusaurus sites, the recommended approach is to use API routes for authentication handling and React components for frontend interactions. This maintains the static site benefits while adding dynamic authentication features.

**Alternatives considered**:
- NextAuth.js: More commonly used but requires custom Next.js setup that doesn't integrate well with Docusaurus
- Auth.js: Generic solution but less documentation for Docusaurus integration
- Custom authentication: Violates constitution requirement to use Better Auth

## Decision: Session Management Approach

**Rationale**: Cookie-based sessions were chosen as required by the constitution. This provides server-side session management with secure, httpOnly cookies that prevent XSS attacks while maintaining user state across page loads.

## Decision: Navbar State Management

**Rationale**: Using React hooks to detect authentication state and dynamically update navbar items. This provides immediate UI feedback when authentication status changes without requiring page reloads.

## Decision: Protected Route Implementation

**Rationale**: Using middleware-based protection that intercepts requests to sensitive routes. For Docusaurus, this is implemented via custom server middleware or client-side guards depending on deployment configuration.

## Decision: User Metadata Collection

**Rationale**: Storing user background information (software/hardware experience) as metadata during signup. Better Auth supports custom metadata fields that can be retrieved with the user session for personalization.

## Best Practices for Better Auth + Docusaurus Integration

### Security Considerations
- Use environment variables for Better Auth configuration
- Enable CSRF protection in Better Auth
- Validate all user inputs before storing metadata
- Implement rate limiting for auth endpoints

### Performance Considerations
- Optimize session validation for minimal latency impact
- Implement proper caching for session data
- Minimize bundle size impact of auth dependencies

### User Experience Considerations
- Provide immediate feedback during auth operations
- Handle auth state changes smoothly without page flickering
- Implement proper error messaging for auth failures

## Technology Integration Patterns

### API Route Structure
- `/api/auth/signin` - Handle sign-in requests
- `/api/auth/signup` - Handle sign-up with metadata
- `/api/auth/logout` - Handle session termination
- `/api/auth/session` - Get current session status

### Frontend Component Structure
- `AuthProvider` component to manage global auth state
- `withAuth` higher-order component for protected pages
- Custom hooks (`useAuth`, `useSession`) for auth state access

### Data Flow
1. User interacts with auth UI components
2. Frontend calls API routes via fetch requests
3. Better Auth handles authentication logic
4. Session cookies are set/validated
5. Frontend updates UI based on session state

## Middleware Implementation Options

### Option 1: Client-side protection
- Check session status on page load
- Redirect unauthenticated users
- Potential for brief content flickering

### Option 2: Server-side protection
- Use server middleware to validate sessions
- Prevent content delivery to unauthorized users
- Better security but requires custom server setup

**Selected**: A hybrid approach using client-side protection with server-side validation for critical operations.

## Constitution Compliance Verification

All decisions align with the constitution requirements:
- Uses Better Auth as mandated
- Implements cookie-based sessions as required
- Follows TypeScript standards
- Integrates with Docusaurus structure
- Includes proper error handling and validation

---
# Data Model: Better Auth Integration for Docusaurus Platform

## User Entity

**Description**: Core user identity managed by Better Auth

**Fields**:
- `id` (string): Unique user identifier generated by Better Auth
- `email` (string): User's email address (unique, required)
- `createdAt` (Date): Account creation timestamp
- `updatedAt` (Date): Last modification timestamp
- `emailVerified` (boolean): Email verification status

## UserMetadata Entity

**Description**: User background information collected during onboarding

**Fields**:
- `softwareBackground` (string): User's software development experience and known languages
- `hardwareExperience` (string): User's hardware/robotics experience and boards used
- `learningTrack` (string): User's preferred learning path (SOFTWARE_ONLY | HARDWARE_ONLY | FULL_ROBOTICS)
- `skillLevel` (string): User's skill level (BEGINNER | INTERMEDIATE | ADVANCED)

**Relationships**:
- One-to-One with User entity (stored as metadata in Better Auth)

## AuthSession Entity

**Description**: Session information managed by Better Auth with cookie-based persistence

**Fields**:
- `sessionId` (string): Unique session identifier
- `userId` (string): Reference to User entity
- `expiresAt` (Date): Session expiration timestamp
- `createdAt` (Date): Session creation timestamp
- `ipAddress` (string): IP address of session origin (for security)
- `userAgent` (string): Browser/device information (for security)

## ProtectedRoute Entity

**Description**: Configuration for routes requiring authentication

**Fields**:
- `path` (string): Route path pattern (e.g., "/dashboard", "/docs/advanced/*")
- `name` (string): Human-readable route name
- `description` (string): Purpose of the protected route
- `redirectPath` (string): Where to redirect unauthenticated users

## DashboardContent Entity

**Description**: Personalized content displayed on user dashboard

**Fields**:
- `userId` (string): Reference to User entity
- `welcomeMessage` (string): Personalized welcome message
- `backgroundDisplay` (string): Formatted user background information
- `recommendedModules` (array): List of recommended learning modules based on profile
- `lastAccessed` (Date): Timestamp of last dashboard access

## FormSubmission Entity

**Description**: Temporary data for onboarding form processing

**Fields**:
- `userId` (string): Reference to User entity
- `formData` (object): Raw form data submitted during onboarding
- `status` (string): Processing status (PENDING | COMPLETED | FAILED)
- `submittedAt` (Date): Timestamp of form submission
- `processedAt` (Date): Timestamp when processing completed

## Validation Rules

### User Entity
- Email must be valid email format
- Email must be unique across all users
- Required fields must not be empty

### UserMetadata Entity
- softwareBackground and hardwareExperience must be strings
- learningTrack must be one of: SOFTWARE_ONLY, HARDWARE_ONLY, FULL_ROBOTICS
- skillLevel must be one of: BEGINNER, INTERMEDIATE, ADVANCED

### AuthSession Entity
- Session must not be expired when accessed
- Session must correspond to valid user account

## State Transitions

### User Registration Flow
1. User enters credentials → Account Created (pending metadata)
2. User completes onboarding → Metadata Added → Account Active
3. User signs in → Session Created → Authenticated State

### Authentication Flow
1. Guest → Unauthenticated State
2. Successful sign-in → Authenticated State → Session Active
3. Session expires/logout → Unauthenticated State

### Profile Completion
1. New user → Incomplete Profile
2. Onboarding completed → Complete Profile
3. Profile updated → Updated Profile

---
openapi: 3.0.0
info:
  title: Better Auth Integration API
  description: API for authentication, authorization, and user management in the Docusaurus Physical AI platform
  version: 1.0.0
servers:
  - url: /api/auth
    description: Authentication API endpoints

paths:
  /signup:
    post:
      summary: Register a new user
      description: Creates a new user account with provided credentials and background information
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required:
                - email
                - password
                - softwareBackground
                - hardwareExperience
              properties:
                email:
                  type: string
                  format: email
                  description: User's email address
                password:
                  type: string
                  format: password
                  description: User's password (min 8 characters)
                softwareBackground:
                  type: string
                  description: User's software development background and known languages
                hardwareExperience:
                  type: string
                  description: User's hardware/robotics experience and boards used
                learningTrack:
                  type: string
                  enum: [SOFTWARE_ONLY, HARDWARE_ONLY, FULL_ROBOTICS]
                  description: User's preferred learning track
                skillLevel:
                  type: string
                  enum: [BEGINNER, INTERMEDIATE, ADVANCED]
                  description: User's skill level
      responses:
        '200':
          description: User successfully created
          content:
            application/json:
              schema:
                type: object
                properties:
                  user:
                    $ref: '#/components/schemas/User'
                  session:
                    $ref: '#/components/schemas/Session'
        '400':
          description: Invalid input or user already exists
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
        '500':
          description: Internal server error
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'

  /signin:
    post:
      summary: Authenticate user
      description: Authenticates user with email and password, creates session
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required:
                - email
                - password
              properties:
                email:
                  type: string
                  format: email
                password:
                  type: string
                  format: password
      responses:
        '200':
          description: Authentication successful
          content:
            application/json:
              schema:
                type: object
                properties:
                  session:
                    $ref: '#/components/schemas/Session'
        '400':
          description: Invalid credentials
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
        '401':
          description: Authentication failed
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'

  /logout:
    post:
      summary: End user session
      description: Invalidates current user session
      responses:
        '200':
          description: Successfully logged out
        '401':
          description: Not authenticated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'

  /session:
    get:
      summary: Get current session
      description: Retrieves information about the current user session
      responses:
        '200':
          description: Current session information
          content:
            application/json:
              schema:
                type: object
                properties:
                  user:
                    $ref: '#/components/schemas/User'
                  session:
                    $ref: '#/components/schemas/Session'
        '401':
          description: Not authenticated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'

components:
  schemas:
    User:
      type: object
      properties:
        id:
          type: string
          description: Unique user identifier
        email:
          type: string
          format: email
        createdAt:
          type: string
          format: date-time
        updatedAt:
          type: string
          format: date-time
        emailVerified:
          type: boolean
        metadata:
          $ref: '#/components/schemas/UserMetadata'

    UserMetadata:
      type: object
      properties:
        softwareBackground:
          type: string
          description: User's software development background
        hardwareExperience:
          type: string
          description: User's hardware/robotics experience
        learningTrack:
          type: string
          enum: [SOFTWARE_ONLY, HARDWARE_ONLY, FULL_ROBOTICS]
        skillLevel:
          type: string
          enum: [BEGINNER, INTERMEDIATE, ADVANCED]

    Session:
      type: object
      properties:
        sessionId:
          type: string
          description: Unique session identifier
        userId:
          type: string
          description: Reference to user
        expiresAt:
          type: string
          format: date-time
          description: Session expiration timestamp
        createdAt:
          type: string
          format: date-time
          description: Session creation timestamp

    ErrorResponse:
      type: object
      properties:
        error:
          type: string
          description: Error message
        code:
          type: string
          description: Error code

---
# Quickstart: Better Auth Integration for Docusaurus Platform

## Prerequisites

- Node.js LTS installed
- Docusaurus project set up
- Git repository initialized

## Installation

1. Install Better Auth dependency:
   ```bash
   npm install better-auth
   ```

2. Verify installation in package.json:
   ```json
   {
     "dependencies": {
       "better-auth": "^0.x.x"
     }
   }
   ```

## Configuration

1. Create API routes directory: `api/auth/`

2. Create the following API route files:
   - `api/auth/signin.ts`
   - `api/auth/signup.ts`
   - `api/auth/logout.ts`

3. Configure Better Auth in your application with cookie-based sessions.

## Frontend Setup

1. Create auth pages in `src/pages/auth/`:
   - `signin.tsx`
   - `signup.tsx`
   - `dashboard.tsx`

2. Update navbar configuration to include auth-aware behavior.

## Protected Routes

1. Implement middleware to protect:
   - `/dashboard`
   - `/docs/advanced/*`

2. Redirect unauthenticated users to `/auth/signin`.

## Testing

1. Run the Docusaurus development server:
   ```bash
   npm run start
   ```

2. Test user registration flow with background information collection.

3. Verify protected routes redirect unauthenticated users.

4. Confirm dashboard displays personalized user information.

## Deployment

1. Ensure environment variables are configured for production.

2. Verify cookie settings work in production environment.

3. Test complete auth flow on deployed site.

## Outcome

- ✅ Impact: Created comprehensive implementation plan for Better Auth integration with Docusaurus platform
- 🧪 Tests: No tests created (planning phase)
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, contracts/auth-api.yaml
- 🔁 Next prompts: Ready for task generation phase with /sp.tasks
- 🧠 Reflection: Plan covers all required aspects of auth implementation with proper architecture and constitution compliance

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable for planning
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Proceed to task generation phase
