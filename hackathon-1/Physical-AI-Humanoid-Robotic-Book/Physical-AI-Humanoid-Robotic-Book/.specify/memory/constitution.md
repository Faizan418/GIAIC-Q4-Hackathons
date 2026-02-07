<!-- SYNC IMPACT REPORT
Version change: 1.1.0 → 1.2.0 (added Auth & Onboarding specification)
List of modified principles: None
Added sections: Authentication & Onboarding Principles
Removed sections: None
Templates requiring updates: ⚠ .specify/templates/plan-template.md, ⚠ .specify/templates/spec-template.md, ⚠ .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# AI / Spec-Driven Book: Physical AI & Humanoid Robotics Constitution

## Core Principles

### Docusaurus-First Content Structure
All content must follow Docusaurus docs folder structure with sequential numbering. Use .md or .mdx files with consistent heading hierarchy (# for module, ## for chapters, ### for sections).

### Spec-Driven Development Governance
All content creation and updates must align with /sp.specify, /sp.plan, and /sp.constitution. Change control requires updating these governance documents when scope, tools, or structure changes.

### Quality and Verification (NON-NEGOTIABLE)
All factual claims must be traceable through verified sources. Code runs as documented, simulations respect real-world physics, and learning outcomes align with labs and assessments. Zero plagiarism tolerance.

### Tooling Mandate
Mandatory use of Docusaurus (TypeScript) for book authoring, GitHub Pages for deployment, Spec-Kit Plus for specification governance, and Claude Code for assisted authoring with human validation required.

### Academic Standards
Citation style: APA. Engineering clarity for CS/Robotics audience. Physical realism over theory-only claims. Accuracy through verified sources.

### Content Structure Compliance
All files must follow Docusaurus docs folder structure with sequential and descriptive filenames to maintain order.

## Technology and Standards Requirements

Mandatory tooling: Docusaurus (TypeScript), GitHub Pages, Spec-Kit Plus, Claude Code. Standards: APA citation style, reproducible simulations and code, accuracy through verified sources.

## Development Workflow

Use Spec-Kit Plus commands (/sp.specify, /sp.plan, /sp.tasks) for specification governance. Human validation required for Claude Code assistance. Follow Docusaurus authoring guidelines with proper file structure.

## Governance

Constitution supersedes all other practices. Amendments require documentation via /sp.constitution command. All content must comply with this constitution. Changes to scope, tools, or structure require updating /sp.constitution, /sp.specify, and /sp.plan.

**Version**: 1.0.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14

# Physical AI & Humanoid Robotics – AI-Native Textbook Constitution

## Core Principles

### Docusaurus-First Content Structure
All content must follow Docusaurus docs folder structure with sequential numbering. Use .md or .mdx files with consistent heading hierarchy (# for module, ## for chapters, ### for sections). Additionally, content must enable semantic search capabilities by supporting Cohere embedding generation and Qdrant vector storage.

### Spec-Driven Development Governance
All content creation and updates must align with /sp.specify, /sp.plan, and /sp.constitution. Change control requires updating these governance documents when scope, tools, or structure changes.

### Quality and Verification (NON-NEGOTIABLE)
All factual claims must be traceable through verified sources. Code runs as documented, simulations respect real-world physics, and learning outcomes align with labs and assessments. Zero plagiarism tolerance. All RAG responses must be traceable to specific book content with proper citations.

### Tooling Mandate
Mandatory use of Docusaurus (TypeScript) for book authoring, GitHub Pages for deployment, Spec-Kit Plus for specification governance, and Claude Code for assisted authoring with human validation required. For RAG functionality, integrate FastAPI + Uvicorn for backend, Cohere for embeddings, Qdrant for vector search, Neon Postgres for metadata, and OpenAI ChatKit for LLM generation.

### Academic Standards
Citation style: APA. Engineering clarity for CS/Robotics audience. Physical realism over theory-only claims. Accuracy through verified sources.

### Content Structure Compliance
All files must follow Docusaurus docs folder structure with sequential and descriptive filenames to maintain order.

## RAG Chatbot Module Principles

### Backend Architecture Governance

#### FastAPI + Uvicorn Standards
All backend endpoints must use FastAPI with Uvicorn ASGI server. Implement proper type hints, async/await patterns, and structured logging. API endpoints for `/query` and `/highlight-query` must follow RESTful principles with proper error handling.

#### Cohere Embedding Pipeline
All book content must be processed through Cohere embedding pipeline with proper chunking strategy. Embeddings must be stored in Qdrant vector database with corresponding metadata in Neon Postgres. Implement proper error handling and retry logic for API calls.

#### Data Management and Storage
Use Neon Serverless Postgres for book content and metadata storage. Use Qdrant Cloud for vector storage. Implement proper data validation, backup strategies, and migration procedures. All sensitive data must be stored in environment variables.

### Frontend Integration Standards

#### Docusaurus Chat Widget
The RAG chat widget must be seamlessly integrated into Docusaurus frontend with proper React/TypeScript implementation. Support text selection and context-aware multi-turn conversations. Follow accessibility standards and responsive design principles.

#### User Experience Requirements
Provide fast, context-aware responses with proper loading states. Implement proper error handling for network issues. Maintain conversation history and support text highlighting for targeted queries.

## Authentication & Onboarding Principles

### Authentication Stack Governance
All authentication must use **Next.js (App Router) + TypeScript** with **Better Auth** as the primary provider. Session strategy must be cookie-based sessions with profile storage in application database (separate from auth). Auth is responsible **only for identity**. Personalization data is handled by the application.

### Signup & Signin Flow Requirements
The signup flow must follow: 1) User enters email + password, 2) Account created via Better Auth, 3) User redirected to onboarding, 4) Background questions collected, 5) Profile persisted, 6) Redirect to dashboard. The signin flow must follow: 1) User authenticates, 2) Session restored, 3) If profile missing → onboarding, 4) Else → dashboard.

### User Background Profiling (Mandatory)
User onboarding must collect mandatory background information: Software background (skill level: BEGINNER | INTERMEDIATE | ADVANCED, known languages: Python, JavaScript, C++, Other), Hardware background (electronics experience: NONE | BASIC | INTERMEDIATE | ADVANCED, boards used: Arduino, ESP32, Raspberry Pi), and Learning Intent (track: SOFTWARE_ONLY, HARDWARE_ONLY, FULL_ROBOTICS). This data is used to personalize content visibility and sequencing.

### Better Auth Configuration Standards
Better Auth must be configured with emailAndPassword enabled and cookie-based sessions. Configuration must follow security best practices and integrate properly with the Next.js application router.

### Animated UI Constitution (CSS)
All authentication and onboarding UI must follow specific design principles: minimal, motion-driven feedback, no third-party animation libraries, hardware-accelerated transforms only. Form containers must use cardEnter animation (600ms ease-out), input fields must have focus states with border highlighting, buttons must have hover/active states with transforms, and loading states must include spin animations.

#### Form Container Animation
Auth cards must have backdrop-filter blur effect, rounded corners, and enter animation with opacity and transform transitions.

#### Input Field Animation
Input fields must have proper padding, border styling, background with transparency, and focus states with border color changes and box shadows.

#### Button Animation
Buttons must have gradient backgrounds, hover states with translateY and box shadows, and active states with scale transforms.

#### Loading State Animation
Loading buttons must have disabled pointer events, opacity changes, and spinning indicators.

### Enforcement Rules
Signup and Signin must use identical animation tokens. No inline styles allowed. Motion must not exceed 600ms. All auth screens must be keyboard accessible.

### Personalization Contract
Content engines must consume profile data before rendering: BEGINNER → show foundations, HARDWARE_ONLY → hide AI/ML, FULL_ROBOTICS → unlock ROS, sensors, control loops.

### GitHub Actions Workflow Requirements
Authentication-related workflows must include: `ci.yml` for build and type safety checks, `auth-check.yml` for constitution compliance verification, and `deploy.yml` for production deployment. All workflows must pass before merge or deploy, and no direct commits to main without workflows.

## Technology and Standards Requirements

Mandatory tooling: Docusaurus (TypeScript), GitHub Pages, Spec-Kit Plus, Claude Code. For RAG functionality: FastAPI, Uvicorn, Cohere, Qdrant, Neon Postgres, OpenAI ChatKit. For authentication: Next.js, Better Auth, TypeScript. Standards: APA citation style, reproducible simulations and code, accuracy through verified sources.

## Development Workflow

Use Spec-Kit Plus commands (/sp.specify, /sp.plan, /sp.tasks) for specification governance. Human validation required for Claude Code assistance. Follow Docusaurus authoring guidelines with proper file structure. Implement CI/CD with automated testing, linting, and deployment for both frontend and backend components. Authentication flows must follow the defined signup/signin patterns with proper user onboarding.

## Governance

Constitution supersedes all other practices. Amendments require documentation via /sp.constitution command. All content must comply with this constitution. Changes to scope, tools, or structure require updating /sp.constitution, /sp.specify, and /sp.plan.

**Version**: 1.2.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-18
