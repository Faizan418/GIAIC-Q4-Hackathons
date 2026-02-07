# ADR-0002: Curriculum Architecture and Module Progression

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-14
- **Feature:** 001-curriculum-overview
- **Context:** The educational robotics curriculum needs to be structured to progressively build student capabilities from foundational concepts to advanced integration. The curriculum must balance theoretical understanding with practical application while ensuring students can successfully complete an integrated capstone project that demonstrates mastery of all concepts. This requires careful sequencing of modules and clear definition of prerequisite relationships.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Structure the curriculum as four progressive modules with clear prerequisite relationships and a capstone integration system:

- Module 1: ROS 2 as the robotic nervous system (foundational communication layer)
- Module 2: Digital twins and physics-based simulation (virtual testing and validation environment)
- Module 3: Perception, navigation, and learning with Isaac (sensing and mobility capabilities)
- Module 4: Vision-Language-Action and cognitive robotics (advanced integration with LLMs)

Each module builds upon previous knowledge with Module 1 as the essential foundation for all subsequent modules. The capstone system integrates components from all modules into a unified voice-controlled robotic system with LLM-based task planning and sim-to-real capabilities.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Clear progression from foundational to advanced concepts ensures students build competency systematically
- Prerequisite relationships prevent knowledge gaps and enable effective learning
- Capstone-first design gives students a clear target and motivation throughout the course
- Modularity allows for flexible pacing and potential reuse of modules in different contexts
- Integration focus prepares students for real-world robotics development challenges
- Sim-to-real approach bridges the gap between simulation and physical robotics

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Students who struggle with Module 1 (ROS 2) may face challenges throughout the entire curriculum
- Linear progression may not accommodate different learning speeds or backgrounds effectively
- Complex capstone system may be overwhelming for some students
- Dependencies between modules create scheduling constraints for educators
- Late introduction of advanced topics (Module 4) may delay student engagement with cognitive robotics
- High cognitive load in final modules due to integration of all previous concepts

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

Alternative A: Spiral curriculum approach (revisit topics at increasing levels of complexity)
- Why rejected: Would make it harder to define clear module boundaries and prerequisite relationships

Alternative B: Parallel modules (learn all topics simultaneously)
- Why rejected: Would increase cognitive load and make integration more difficult to achieve

Alternative C: Capstone at the beginning with reverse learning (learn components as needed)
- Why rejected: Would make it difficult to build foundational knowledge systematically

Alternative D: Independent modules with no prerequisite relationships
- Why rejected: Would prevent the integrated learning approach and capstone system integration

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: ../specs/001-curriculum-overview/spec.md
- Implementation Plan: ../specs/001-curriculum-overview/plan.md
- Related ADRs: ADR-0001 Multi-Feature Coordination Strategy
- Evaluator Evidence: ../history/prompts/001-curriculum-overview/0002-curriculum-overview-plan.plan.prompt.md
