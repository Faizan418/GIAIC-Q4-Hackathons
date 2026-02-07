# Strict Layer Guidelines

Dependency flow: Top → Bottom only (never reverse)

1. **apps/**  
   Can import: Everything below  
   Responsibility: Wiring, routing, entry points only

2. **packages/features/**  
   Can import: domain, data-access, ui, utils  
   Responsibility: Business flows and use cases

3. **packages/domain/**  
   Can import: utils only  
   Responsibility: Pure business rules, entities, value objects

4. **packages/data-access/**  
   Can import: domain, utils  
   Responsibility: Databases, APIs, external services

5. **packages/ui/**  
   Can import: utils, spec-kit  
   Responsibility: Components, styling, design system

6. **packages/utils/** & **config/**  
   Can import: Nothing  
   Responsibility: Generic helpers, shared configurations

Enforcement:
- ESLint plugin-import with layer rules
- TypeScript project references
- Regular dependency audits