# ADR-0005: Better Auth Integration for Session Management

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-19
- **Feature:** 008-auth-navbar-dropdown
- **Context:** Need to implement secure session management for authentication-aware navbar that integrates with Better Auth for user identity and provides clean separation between authentication logic and UI rendering.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Use Better Auth's session context (`useSession` hook) for authentication state management in the navbar component with the following components:
- Session context integration using Better Auth's `useSession` hook
- Authentication state handling with three states: `loading`, `authenticated`, `unauthenticated`
- Proper logout implementation using Better Auth's `signOut()` function
- Default to unauthenticated state on session errors for security
- Loading state handling that prevents UI flickering

## Consequences

### Positive

- Leverages a well-maintained, security-focused authentication library
- Provides clean separation between authentication logic and UI rendering
- Follows security best practices for session management
- Integrates properly with Better Auth's ecosystem and security features
- Reduces custom security implementation complexity
- Maintains consistency with project's authentication approach (as per constitution)

### Negative

- Creates dependency on Better Auth library and its API
- May require updates if Better Auth changes their API
- Learning curve for Better Auth specific patterns
- Potential vendor lock-in to Better Auth ecosystem

## Alternatives Considered

Alternative 1: Manual API calls to check authentication status - Rejected because this would be less efficient and potentially less secure than using the established Better Auth session management.

Alternative 2: Global state management (Redux/Zustand) for authentication state - Rejected because this would add unnecessary complexity for a simple session check, and Better Auth's built-in context is designed for this purpose.

Alternative 3: Custom authentication implementation - Rejected because this would be more complex, less secure, and reinvent existing solutions that Better Auth provides.

## References

- Feature Spec: ../specs/008-auth-navbar-dropdown/spec.md
- Implementation Plan: ../specs/008-auth-navbar-dropdown/plan.md
- Related ADRs: ADR-0004 (Docusaurus NavbarItem Extension for Authentication)
- Evaluator Evidence: ../specs/008-auth-navbar-dropdown/research.md, ../specs/008-auth-navbar-dropdown/data-model.md
