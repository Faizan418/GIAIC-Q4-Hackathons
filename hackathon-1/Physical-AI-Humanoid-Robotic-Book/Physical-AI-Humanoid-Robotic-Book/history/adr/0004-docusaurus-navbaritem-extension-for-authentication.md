# ADR-0004: Docusaurus NavbarItem Extension for Authentication

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-19
- **Feature:** 008-auth-navbar-dropdown
- **Context:** Need to implement an authentication-aware navbar dropdown in Docusaurus that displays different content based on user authentication status while maintaining compatibility with Docusaurus native behavior and avoiding custom event listeners.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Use the official Docusaurus NavbarItem extension mechanism to create a custom authentication-aware dropdown component that integrates with Better Auth session management. The solution includes:
- Custom NavbarItem component at `src/theme/NavbarItem/AuthDropdown.tsx`
- Docusaurus native CSS classes for consistent styling (`navbar__item`, `dropdown`, `dropdown--hoverable`, `navbar__link`, `dropdown__menu`, `dropdown__link`)
- Proper HTML semantic structure with `<ul> > <li> > <a/button>` pattern
- Loading state handling that prevents rendering during authentication checks

## Consequences

### Positive

- Maintains compatibility with Docusaurus theme updates and native dropdown behavior
- Follows Docusaurus best practices and architectural patterns
- Ensures consistent styling with the rest of the navbar
- Prevents conflicts with Docusaurus dropdown behavior (hover, click, keyboard navigation)
- Maintains accessibility standards by using semantic HTML
- Long-term maintainability through framework-native patterns

### Negative

- Requires understanding of Docusaurus theme extension mechanism
- Limited to Docusaurus-specific customization patterns
- Tightly coupled to Docusaurus framework for navbar behavior
- May require updates if Docusaurus changes their NavbarItem API

## Alternatives Considered

Alternative 1: Direct navbar override using `@theme-original/Navbar` - Rejected because this would break Docusaurus native behavior and cause compatibility issues with theme updates.

Alternative 2: Custom dropdown implementation with useState and document.addEventListener - Rejected because this violates the specification requirement to avoid custom dropdown state management and could conflict with Docusaurus native behavior.

Alternative 3: Third-party dropdown library integration - Rejected because this would add unnecessary complexity and might not integrate properly with Docusaurus styling conventions.

## References

- Feature Spec: ../specs/008-auth-navbar-dropdown/spec.md
- Implementation Plan: ../specs/008-auth-navbar-dropdown/plan.md
- Related ADRs: ADR-0005 (Better Auth Integration for Session Management)
- Evaluator Evidence: ../specs/008-auth-navbar-dropdown/research.md, ../specs/008-auth-navbar-dropdown/data-model.md
