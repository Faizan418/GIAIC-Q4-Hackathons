---
name: acceptance-criteria
description: Writes comprehensive, unambiguous Given-When-Then acceptance criteria that fully define when a user story is considered complete.
---

# Acceptance Criteria Skill

This skill creates clear, testable acceptance criteria using the Gherkin-style Given-When-Then format.

## Standard Format

```markdown
#### Acceptance Criteria

**Scenario 1: Happy Path**
- Given [preconditions]
- When [user action]
- Then [expected outcome]
- And [additional outcome]

**Scenario 2: Edge Case**
- Given [different preconditions]
- When [action]
- Then [outcome]
- And [validation]

**Scenario 3: Error Case**
- Given [error condition]
- When [action]
- Then [error handling]
- And [user feedback]
Best Practices

Cover happy path, edge cases, error cases
Be specific and unambiguous
Avoid implementation details
Include validation of UI, API response, database state
Cover accessibility and performance where relevant
Write from user's perspective

Common Elements to Verify

UI renders correctly
Data is saved correctly
API returns correct status/response
Error messages are user-friendly
Loading states are handled
Responsive on mobile/desktop

Always ensure criteria are testable by QA.