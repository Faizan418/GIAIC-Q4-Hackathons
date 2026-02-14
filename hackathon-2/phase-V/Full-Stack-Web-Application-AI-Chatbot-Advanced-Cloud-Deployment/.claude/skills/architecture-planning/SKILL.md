---
name: architecture-planning
description: Designs a clean, scalable, and maintainable monorepo architecture using proven patterns (feature-sliced + layered hybrid), strict dependency enforcement, boundary rules, and comprehensive visualizations.
---

# Architecture Planning Skill

Great architecture prevents technical debt and enables teams to move fast without breaking things.

This skill defines a **tool-agnostic hybrid architecture** combining Feature-Sliced Design (business alignment) and Layered Architecture (technical separation).

## Comprehensive Planning Process

### 1. Domain Analysis
- Identify bounded contexts and core domains
- Map features to domains
- Determine cross-cutting concerns (auth, logging, analytics)

### 2. Recommended Hybrid Pattern
- Primary organization: by feature/domain
- Secondary enforcement: layers within and across features

### 3. Layer Hierarchy (Strict One-Way Dependencies)
1. **apps/** – Orchestration only
2. **packages/features/** – Business use cases
3. **packages/domain/** – Pure business logic, entities
4. **packages/data-access/** – External integrations
5. **packages/ui/** – Presentation components
6. **packages/utils/** & **config/** – Generic helpers (lowest)

### 4. Detailed Structure Example
packages/
├── features/
│   ├── auth/
│   │   ├── login.flow.ts
│   │   ├── register.usecase.ts
│   │   └── components/
│   └── payments/
│       ├── checkout.flow.ts
│       └── stripe.integration.ts
├── domain/
│   ├── user/
│   │   ├── user.entity.ts
│   │   └── user.events.ts
│   └── order/
├── data-access/
│   ├── prisma.client.ts
│   └── external-api/
├── ui/
│   ├── components/Button/
│   ├── theme/
│   └── icons/
└── utils/
├── formatters/
└── validation/
text### 5. Dependency Flow Diagram (Mermaid)

```mermaid
graph TD
    Apps --> Features --> Domain --> DataAccess
    Apps --> UI --> Utils
    Features --> UI
    Domain --> Utils
    DataAccess --> Utils
    All --> Config
6. Enforcement Strategies

ESLint import restriction rules
Project references in TypeScript
Code reviews focused on boundaries
Automated dependency graph checks