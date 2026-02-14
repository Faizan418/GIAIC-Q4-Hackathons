---
name: root-level-guidance
description: Provides detailed guidance on configuring essential root-level files for consistency, performance, and maintainability across any monorepo setup.
---

# Root Level Guidance Skill

Root configuration files define the behavior of your entire monorepo.

## Key Files to Configure

### 1. Root package.json
- private: true
- Standardized scripts
- Shared dev dependencies

### 2. TypeScript Base Config
- Strict mode enabled
- Path mappings
- Composite project references

### 3. ESLint & Prettier
- Shared configuration exported from packages/config

### 4. Workspace Definition
- Defined according to your chosen package manager

### 5. Root Diagram (Mermaid)

```mermaid
graph TD
    RootConfig --> Tooling[Shared Tooling]
    RootConfig --> Workspaces[Package Discovery]
    RootConfig --> Scripts[Common Commands]
    RootConfig --> TypeSafety[TS Config]
    RootConfig --> Quality[Linting & Formatting]