# Universal Shared Tooling Guide (2026 Best Practices)

1. **Root package.json** (Minimal Example):
   json
   {
     "name": "my-monorepo",
     "private": true,
     "scripts": {
       "dev": "your-monorepo-tool run dev",
       "build": "your-monorepo-tool run build",
       "lint": "your-monorepo-tool run lint",
       "test": "your-monorepo-tool run test"
     },
     "devDependencies": {
       "typescript": "^5.5.0",
       "eslint": "^9.0.0",
       "prettier": "^3.3.0"
     }
   }

Shared TypeScript Configuration:
Use root tsconfig.json with "references" for project composition
Enable path aliases for clean imports

ESLint Flat Config (packages/config/eslint.config.js):
Export a shared config that all packages extend

Changesets or Similar:
For automated versioning and changelog generation across packages

Storybook:
Run from packages/ui for component development in isolation