# Research: Frontend Chat Integration

## Decision: Testing Approach
**Chosen**: Jest + React Testing Library + Cypress for end-to-end testing

## Rationale
For the frontend chat integration, we need a comprehensive testing strategy that covers:
- Unit testing for individual React components (ChatWidget, ChatMessage, ChatInput)
- Integration testing for component interactions and API calls
- End-to-end testing for complete user workflows

Jest + React Testing Library is the standard testing stack for React applications and provides excellent utilities for testing React components in isolation. Cypress provides reliable end-to-end testing for user workflows that involve the chat interface.

## Alternatives Considered
1. **Jest + React Testing Library only**: Good for unit and integration tests but lacks comprehensive end-to-end testing
2. **Vitest + React Testing Library**: Faster test execution but less mature ecosystem for React testing
3. **Cypress only**: Good for end-to-end testing but less suitable for component-level unit testing
4. **Playwright**: Alternative to Cypress for end-to-end testing with broader browser support

## Backend API Integration Research
**Decision**: Use fetch API or axios for connecting to backend services

**Rationale**: Both fetch API and axios are standard for making HTTP requests from frontend applications. Since the constitution mentions FastAPI backend with `/query` and `/highlight-query` endpoints, either approach will work well.

**Chosen**: Start with fetch API (built into browsers) with potential migration to axios if advanced features are needed.

## Docusaurus Integration Approach
**Decision**: Create custom React components that integrate with Docusaurus using MDX

**Rationale**: Docusaurus supports custom React components which can be embedded in MDX files or loaded globally. This allows the chat widget to be available across textbook pages while maintaining Docusaurus styling and routing.

**Implementation Options**:
1. Global plugin that adds chat to all pages
2. MDX component that can be selectively added to pages
3. Layout wrapper that includes chat functionality

**Chosen**: Layout wrapper approach to make chat available across all textbook pages consistently.

## Text Highlighting Implementation
**Decision**: Use JavaScript Selection API for text highlighting functionality

**Rationale**: The Selection API is the standard browser API for getting user-selected text. It works reliably across modern browsers and integrates well with React applications.

**Implementation**: Add event listeners for text selection that capture the selected text and trigger the chat interface with the selected content.
