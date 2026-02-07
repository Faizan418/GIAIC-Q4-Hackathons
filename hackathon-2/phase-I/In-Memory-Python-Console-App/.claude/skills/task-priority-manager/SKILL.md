---
name: task-priority-manager
description: Use this agent when managing task priorities and tags including assigning priorities, adding/removing tags, and filtering by priority level or tag.
license: MIT
version: 1.0.0
---

# Task Priority Manager Skill

Task Priority and Tag Management expert specializing in organizing and filtering tasks by their priority and tag attributes.

## Purpose

Provide comprehensive priority and tag management capabilities for tasks including assignment, modification, and filtering operations.

## When to Use

- Setting task priority: "Set priority of task #42 to High"
- Adding tags: "Add 'review' tag to task #5"
- Removing tags: "Remove 'urgent' tag from task #3"
- Filtering by priority: "Show all High priority tasks"
- Filtering by tag: "Show tasks tagged with 'bug'"
- Combined filters: "Show High priority tasks tagged 'review'"

## Priority Levels

Allowed values: **High**, **Medium**, **Low** (case-insensitive)

Priority order (for sorting):
- High = 1 (most important)
- Medium = 2 (default)
- Low = 3 (least important)

## Tag Operations

| Operation | Behavior |
|-----------|----------|
| Add | Append tag if not already present (no duplicates) |
| Remove | Delete specific tag if it exists |
| Set | Replace all tags with new list |

### Tag Rules
- Tags are case-sensitive
- Empty or whitespace-only tags rejected
- Tags must be strings

## Operational Workflows

### Priority Assignment
```
1. Identify target task(s) by ID
2. Receive proposed priority value
3. Validate: is value in [High, Medium, Low]?
4. If invalid: return error with allowed values
5. If valid: apply priority update
6. Confirm with task ID and new priority
```

### Tag Modification
```
1. Identify target task(s)
2. Determine operation type (add/remove/set)
3. Validate tag name if adding/setting
4. Execute with duplicate/existence checks
5. Report result (tags added/removed/set)
```

### Filtering Tasks
```
1. Receive filter criteria (priority and/or tags)
2. Apply priority filter first if specified
3. Apply tag filter second if specified
4. Return matching tasks or empty result
```

## Edge Case Handling

| Scenario | Handling |
|----------|----------|
| No matching tasks | Return empty result set, don't error |
| Invalid priority | List allowed values [High, Medium, Low] |
| Empty tag on add/set | Reject with message |
| Removing non-existent tag | Report no change needed |
| Multiple criteria | Apply all filters (AND logic) |
| Task not found | Report which task is missing |

## Output Standards

| Operation | Output Format |
|-----------|---------------|
| Single priority update | "Task [ID] priority set to High" |
| Batch priority update | "Updated 5 tasks to High priority" |
| Tag add | "Added 'review' tag to Task [ID]" |
| Tag remove | "Removed 'urgent' tag from Task [ID]" |
| Filter results | "Found 3 tasks matching: [list IDs]" |
| No changes needed | "No changes required" |

## Integration

- Work with task objects having `id`, `priority`, `tags` attributes
- Accept tasks by ID, object reference, or task representation
- Return results consistently: task IDs with updated attributes
