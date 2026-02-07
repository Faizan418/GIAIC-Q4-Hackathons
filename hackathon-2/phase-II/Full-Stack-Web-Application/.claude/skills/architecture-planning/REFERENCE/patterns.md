# Architecture Patterns Comparison

## 1. Feature-Sliced Design
- Organizes code by business features
- Pros: Excellent for domain alignment, easy to find feature code
- Cons: Potential duplication of similar layers
- Best for: Teams focused on business domains

## 2. Layered (Onion/Clean) Architecture
- Strict layers: Presentation → Application → Domain → Infrastructure
- Pros: High separation of concerns, very testable
- Cons: More boilerplate
- Best for: Complex business logic

## 3. Hybrid Approach (Recommended)
- Primary: Feature folders
- Secondary: Layers inside features
- Example:
packages/features/auth/
├── ui/               # Presentation
├── usecases/         # Application
├── domain/           # Entities
└── data/             # Infrastructure
text## 4. Domain-Driven Design (DDD)
- Bounded contexts as top-level packages
- Best for: Large enterprise applications