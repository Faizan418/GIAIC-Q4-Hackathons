---
name: markdown-spec-writing
description: Writes clear, professional, well-structured Markdown specifications for features, components, modules, or any technical documentation with consistent formatting, headings, tables, code blocks, and diagrams.
---

# Markdown Spec Writing Skill

This skill creates high-quality, readable, and maintainable Markdown documentation that serves as the single source of truth for any feature or technical decision.

## Guidelines for Excellent Markdown Specs

### 1. Structure Template (Always Follow)
```markdown
# Feature Name

## Overview
Brief description of what this feature does and why it exists.

## Goals
- Business goal 1
- Business goal 2
- Technical goal

## Non-Goals / Out of Scope
- What this feature will NOT do

## User Impact
Who benefits and how?

## Technical Details

### Architecture Diagram (if applicable)
```mermaid
graph TD
    A --> B
    B --> C
Data Flow
Description with sequence or flow diagram.
Components Involved

List of packages/apps affected
New files/modules to be created

Risks & Mitigations

Potential issue → solution

Alternatives Considered

Option 1 → why rejected
Option 2 → why rejected

Open Questions

Question for team discussion

References

Related tickets
Design links
Prior art

text### 2. Best Practices
- Use clear, active voice
- Keep sentences short and direct
- Use consistent heading levels
- Always include Overview, Goals, and Non-Goals
- Use tables for comparisons
- Use code blocks with language tags
- Include Mermaid diagrams for flows/architecture
- Link to related docs/tickets

### 3. Common Spec Types
- Feature specs
- Component specs
- API changes
- Refactoring plans
- Database migrations
- Configuration changes

When writing a spec, always ask for context first if unclear, then output the full Markdown in a code block.

Next: Use **user-story-creation** skill to break the spec into implementable stories.