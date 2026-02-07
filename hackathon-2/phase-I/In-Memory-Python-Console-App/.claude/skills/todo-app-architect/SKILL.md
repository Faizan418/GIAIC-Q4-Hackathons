---
name: todo-app-architect
description: Use this agent when designing an in-memory Python Todo application architecture, defining module responsibilities, and orchestrating agents and subagents for optimal workflow. Invoke during project initialization, feature planning, or agent orchestration.
license: MIT
version: 1.0.0
---

# Todo App Architect Skill

Design the complete in-memory Todo application architecture and orchestrate all subagents for optimal workflow.

## Purpose

This skill provides architectural guidance for building a modular, testable Python Todo console application with clean separation of concerns between CLI, core logic, organization features, and testing.

## When to Use

- Project initialization: Design module structure and assign responsibilities
- Feature planning: Plan how new features fit into the architecture
- Agent orchestration: Coordinate subagents to prevent responsibility overlaps
- Conflict resolution: Realign agent responsibilities when boundaries are violated

## Subagent Organization

### Core Layer
| Subagent | Responsibility |
|----------|----------------|
| `task-model-specialist` | Task data structure (id, title, completed, priority, tags) |
| `in-memory-storage-subagent` | In-memory storage and retrieval |

### UI Layer
| Subagent | Responsibility |
|----------|----------------|
| `menu-flow-subagent` | Menu navigation and input validation |

### Organization Layer
| Subagent | Responsibility |
|----------|----------------|
| `task-priority-manager` | Priority and tag management |
| `search-filter-subagent` | Keyword search and filtering |
| `sorting-subagent` | Task sorting by criteria |

### Quality Layer
| Subagent | Responsibility |
|----------|----------------|
| `todo-testing-agent` | Test execution and validation |
| `todo-edge-case-tester` | Edge case boundary testing |

### Infrastructure Layer
| Subagent | Responsibility |
|----------|----------------|
| `pip-dependency-manager` | Package installation |
| `python-venv-manager` | Virtual environment setup |

## Execution Flow

```
CLI Agent → Menu Flow → Core Handler → Storage/Model → Organization → Testing
                     ↑              ↓
              User Input     State Management
```

## Anti-Patterns to Avoid

| Pattern | Solution |
|---------|----------|
| Core logic in CLI agent | Delegate to `todo-core-handler` |
| Storage logic in organization | Delegate to `in-memory-storage-subagent` |
| Overlapping agent responsibilities | Reassign to single authoritative agent |
| Testing without isolation | Use dedicated testing subagents |

## Quality Checklist

Before confirming architecture changes:

- [ ] Separation of concerns validated
- [ ] In-memory state integrity maintained
- [ ] Each component is testable in isolation
- [ ] No feature creep in agent scopes
- [ ] Clear interface contracts between agents

## Example Usage

```python
# When user says: "Add priority levels and tags to tasks"
# Architect coordinates:
# 1. task-model-specialist: Extend Task with priority/tags fields
# 2. in-memory-storage-subagent: Update storage to handle new fields
# 3. task-priority-manager: Implement priority/tag operations
# 4. menu-flow-subagent: Add priority/tag prompts to menu
# 5. todo-testing-agent: Validate new functionality
```
