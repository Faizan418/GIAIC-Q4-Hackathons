---
name: monorepo-setup
description: Provides a comprehensive, framework-agnostic guide to initialize a scalable monorepo project, covering initial structure, workspace configuration, shared tooling, dependency management, performance considerations, and long-term maintainability best practices.
---

# Monorepo Setup Skill

This skill helps you establish a professional monorepo foundation that works with any modern monorepo tool (Nx, Turborepo, Rush, Lerna, etc.) and any package manager (pnpm, Yarn, npm).

A well-structured monorepo enables seamless code sharing, consistent tooling, atomic commits, and faster cross-team development.

### Detailed Setup Process

1. **Project Requirements Gathering**:
   Always start by asking:
   - What type of applications will exist? (Web frontend, mobile, backend API, admin panel, etc.)
   - How many shared packages are planned? (UI components, utilities, config, types, domain logic, design system)
   - Expected team size and collaboration patterns?
   - Deployment strategy? (Vercel, Netlify, AWS, Docker, etc.)
   - Preferred monorepo tool and package manager? (Let the team decide — this skill remains neutral)

2. **Core Principles (Tool-Agnostic Best Practices)**:
   - Single repository for all related code
   - Clear separation between applications and libraries
   - Shared configuration wherever possible
   - Strict dependency boundaries
   - Consistent tooling across the entire codebase
   - Enforced code quality and formatting

3. **Recommended Initial Folder Structure** (Universal & Scalable):
my-monorepo/
├── apps/
│   ├── web/                   # Primary frontend application
│   │   ├── src/
│   │   ├── public/
│   │   ├── package.json
│   │   └── tsconfig.json
│   ├── api/                   # Backend server or API
│   │   ├── src/
│   │   ├── package.json
│   │   └── tsconfig.json
│   └── mobile/                # Optional: React Native or Flutter app
├── packages/
│   ├── ui/                    # Shared UI components and design system
│   │   ├── src/
│   │   ├── stories/           # Storybook stories
│   │   └── package.json
│   ├── utils/                 # Generic utilities (date, string, validation)
│   ├── config/                # Shared ESLint, Prettier, TypeScript configs
│   ├── types/                 # Shared interfaces and types
│   └── spec-kit/              # Design tokens and component specifications
├── tools/
│   ├── scripts/               # Custom build/deploy/CI scripts
│   └── docker/                # Docker configurations
├── docs/                      # Project documentation
├── .github/                   # Workflows, issue templates
├── package.json               # Root workspace configuration
├── tsconfig.json              # Base TypeScript configuration
├── eslint.config.js           # Shared linting rules (flat config)
├── .prettierrc                # Code formatting
├── .gitignore
├── CLAUDE.md                  # AI collaboration guide
└── README.md                  # Project overview and setup instructions
text**High-Level Dependency Flow (Mermaid Diagram)**:
graph TD
subgraph "Applications"
Web[apps/web]
API[apps/api]
Mobile[apps/mobile]
end
subgraph "Shared Packages"
UI[packages/ui]
Utils[packages/utils]
Config[packages/config]
Types[packages/types]
Spec[packages/spec-kit]
end
Web --> UI & Utils & Spec & Types
API --> Utils & Types & Spec
Mobile --> UI & Utils & Spec
UI --> Utils & Types
All[All Packages & Apps] --> Config
text4. **Essential Shared Tooling Recommendations**:
- TypeScript with strict mode and path mapping
- ESLint (flat config format) + Prettier shared via packages/config
- Versioning strategy (e.g., Changesets or conventional commits)
- Pre-commit hooks (Husky + lint-staged)
- Testing framework (Vitest, Jest, or Playwright)
- UI component documentation (Storybook)
- Design token management (to be configured in spec-kit skill)

5. **Common Pitfalls to Avoid**:
- Mixing application and library code
- Inconsistent tooling across packages
- Lack of dependency boundary enforcement
- No clear ownership model
- Overly deep folder nesting

6. **Next Steps**:
Once the basic structure is in place, proceed to the **architecture-planning** skill to define layers, dependency rules, and long-term scalability.

Refer to the REFERENCE folder for detailed configuration examples and templates.