# ADR-0001: Multi-Feature Coordination Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-14
- **Feature:** 001-curriculum-overview
- **Context:** The project requires managing multiple concurrent features with numeric prefixes that create conflicts in the Spec-Kit Plus tooling. When multiple feature directories share the same numeric prefix (e.g., 001-curriculum-overview, 001-intro-physical-ai, 001-setup-guide), command-line tools that expect unique prefixes fail. This creates a coordination challenge for managing parallel feature development while maintaining tool compatibility.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement a temporary directory renaming strategy during individual feature workflows. When executing Spec-Kit Plus commands that require unique numeric prefixes:

- Temporarily rename other feature directories with the same prefix by adding a "_" prefix (e.g., "_001-intro-physical-ai-temp")
- Execute the required command for the target feature
- Restore the original directory names after completion
- Document this process in feature workflows and team guidelines

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Allows individual feature workflows to complete successfully with existing tooling
- Maintains compatibility with Spec-Kit Plus command-line tools
- Enables parallel feature development without permanent structural changes
- Preserves original directory naming conventions after workflow completion

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Adds complexity to individual feature workflows requiring manual coordination steps
- Risk of forgetting to restore original directory names after workflow completion
- Potential for human error during the renaming process
- Temporary workflow disruption during the renaming/restoring process

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

Alternative A: Modify Spec-Kit Plus tooling to support multiple features with same prefix
- Why rejected: Would require significant tooling changes and could break existing workflows

Alternative B: Use different numeric prefixes for each feature (e.g., 001, 002, 003)
- Why rejected: Would require restructuring existing features and could create confusion about feature relationships

Alternative C: Sequential feature development (one at a time)
- Why rejected: Would significantly slow down development and prevent parallel work streams

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: ../specs/001-curriculum-overview/spec.md
- Implementation Plan: ../specs/001-curriculum-overview/plan.md
- Related ADRs: None
- Evaluator Evidence: ../history/prompts/001-curriculum-overview/0002-curriculum-overview-plan.plan.prompt.md
