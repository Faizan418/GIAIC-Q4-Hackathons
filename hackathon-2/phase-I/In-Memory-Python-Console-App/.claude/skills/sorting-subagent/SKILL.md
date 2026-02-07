---
name: sorting-subagent
description: Use this agent when sorting a collection of tasks by different criteria. Sort by title, priority, creation date, or due date with configurable direction.
license: MIT
version: 1.0.0
---

# Sorting Subagent Skill

Task Sorting Specialist for implementing and executing sorting operations on task collections with precision and consistency.

## Purpose

Provide reliable task sorting capabilities based on various criteria including title, priority, creation date, and due date.

## When to Use

- Alphabetical sorting: "Show tasks sorted by name"
- Priority sorting: "Sort by priority, high to low"
- Creation date sorting: "Order by creation date, newest first"
- Due date sorting: "What's due soonest?"
- Default sorting: "Set default sort order for views"

## Sorting Methods

### Sort by Title
- Case-insensitive alphabetical ordering
- Handle mixed case titles consistently
- Preserve original casing in output

### Sort by Priority
Priority order (ascending numerical = higher user priority):
- High = 1 (most important)
- Medium = 2 (default)
- Low = 3 (least important)

Default direction: High → Low

### Sort by Created At
- Default: newest → oldest (reverse chronological)
- Accept `reverse` parameter for ascending order

### Sort by Due Date
- Tasks without due dates appear last
- Default: earliest due date first (soonest first)

## Implementation Example

```python
from datetime import datetime

class TaskSorter:
    PRIORITY_ORDER = {"High": 1, "Medium": 2, "Low": 3}

    @staticmethod
    def sort_by_title(tasks, reverse=False):
        return sorted(tasks, key=lambda t: t.title.lower(), reverse=reverse)

    @staticmethod
    def sort_by_priority(tasks, reverse=False):
        return sorted(tasks, key=lambda t: TaskSorter.PRIORITY_ORDER.get(
            t.priority, 999), reverse=reverse)

    @staticmethod
    def sort_by_created_at(tasks, reverse=True):
        return sorted(tasks, key=lambda t: t.created_at, reverse=reverse)

    @staticmethod
    def sort_by_due_date(tasks, reverse=False):
        return sorted(tasks, key=lambda t: (t.due_date is None, t.due_date), reverse=reverse)
```

## Output Requirements

For each sorting request:
1. Return sorted list of task objects
2. Include metadata:
   - Sort criterion used
   - Direction (ascending/descending)
   - Number of tasks sorted
3. If input empty/invalid: return empty list with message

## Quality Assurance

- Validate sorting criterion exists on tasks
- Handle missing values gracefully (null due_date)
- Ensure sorting is stable
- Confirm sort applied by spot-checking elements

## Error Handling

| Scenario | Handling |
|----------|----------|
| Unknown criterion | Return original collection with warning |
| Malformed task objects | Log error, return empty list |
| Empty input | Return empty list with clear message |
| Missing due_date | Place tasks last in due date sort |

## Sorting Direction Summary

| Criterion | Default | Ascending | Descending |
|-----------|---------|-----------|------------|
| Title | A-Z | A-Z | Z-A |
| Priority | High→Low | Low→High | High→Low |
| Created At | Newest→Oldest | Oldest→Newest | Newest→Oldest |
| Due Date | Earliest→Latest | Earliest→Latest | Latest→Earliest |
