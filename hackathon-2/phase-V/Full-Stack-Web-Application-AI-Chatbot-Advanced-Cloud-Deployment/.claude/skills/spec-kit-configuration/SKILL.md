---
name: spec-kit-configuration
description: Establishes a centralized design token and component specification system for visual and thematic consistency across all applications.
---

# Spec-Kit Configuration Skill

A single source of truth for design decisions dramatically improves consistency and reduces UI debt.

## Process

### 1. Create Dedicated Package
- packages/spec-kit or packages/design-system/tokens

### 2. Comprehensive Structure
packages/spec-kit/
├── tokens/
│   ├── core/                  # Colors, spacing, typography
│   ├── semantic/              # Primary, success, danger
│   ├── components/            # Button, card, input specs
│   └── themes/                # light, dark, brand variations
├── transforms/                # Build configurations
├── outputs/                   # Generated CSS, TS, Tailwind
├── index.ts                   # Typed exports
└── package.json
text### 3. Sync Strategy
- Export from Figma (manual or automated)
- Version control all tokens
- Generate multiple formats (CSS variables, TypeScript, Tailwind config)

### 4. Integration Flow (Mermaid)

```mermaid
graph LR
    DesignTool --> TokenExport
    TokenExport --> GitRepo
    GitRepo --> BuildProcess
    BuildProcess --> GeneratedOutputs
    GeneratedOutputs --> AllApps[All Applications]
    GeneratedOutputs --> UIComponents