---
name: user-story-creation
description: Converts requirements and specifications into well-written, INVEST-compliant user stories with clear titles, descriptions, and actionable tasks.
---

# User Story Creation Skill

This skill transforms approved specs into actionable, well-defined user stories that follow the INVEST principle (Independent, Negotiable, Valuable, Estimable, Small, Testable).

## Standard User Story Template

```markdown
### As a [type of user],
### I want [some goal]
### So that [some reason/business value]

#### Description
Detailed explanation of the behavior.

#### Acceptance Criteria
Given-When-Then format (see next skill).

#### Tasks
- [ ] Technical task 1
- [ ] Technical task 2
- [ ] Update documentation

#### Notes
- Dependencies
- Edge cases
- Performance considerations
INVEST Guidelines

Independent: Can be developed alone
Negotiable: Details can be discussed
Valuable: Delivers clear user/business value
Estimable: Team can estimate effort
Small: Fits in one sprint
Testable: Clear pass/fail criteria

Best Practices

One story = one discrete behavior
Avoid technical language in the "As a/I want/So that" part
Break large features into multiple stories
Include tasks for frontend, backend, testing, docs
Link back to the parent spec

Always output stories in clean Markdown format.