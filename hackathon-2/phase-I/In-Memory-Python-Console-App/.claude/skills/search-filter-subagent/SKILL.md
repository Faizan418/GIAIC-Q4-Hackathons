---
name: search-filter-subagent
description: Use this agent when implementing search and filter functionality on task lists. Implement keyword search, status filtering, priority filtering, and tag-based filtering.
license: MIT
version: 1.0.0
---

# Search Filter Subagent Skill

Task Query Specialist for implementing efficient, reliable search and filter operations on task collections.

## Purpose

Provide powerful search and filtering capabilities that allow users to find specific tasks based on keywords, status, priority, and tags.

## When to Use

- Keyword search: "Find tasks containing 'meeting'"
- Status filtering: "Show only completed tasks"
- Priority filtering: "Show high priority items"
- Tag filtering: "Filter tasks by 'work' or 'personal'"
- Combined filtering: "Completed high priority tasks"

## Core Methods

### search_by_keyword(tasks, keyword)
- Perform case-insensitive substring matching
- Search across task titles
- Return all tasks where keyword appears in title
- Handle empty keyword by returning all tasks

### filter_by_status(tasks, completed)
- Filter tasks based on boolean completed status
- Handle True (completed) and False (pending)
- Return empty list if no tasks match

### filter_by_priority(tasks, priority)
- Filter tasks matching exact priority level
- Accept: "high", "medium", "low" (case-insensitive)
- Return empty list if no tasks have priority

### filter_by_tag(tasks, tag)
- Check if tag exists in task's tags collection
- Case-insensitive tag matching
- Handle tasks with no tags appropriately

## Implementation Standards

- Use list comprehensions for clean, Pythonic filtering
- Ensure all methods are static (no instance state)
- Preserve original task objects in results
- Handle None inputs gracefully
- Return new lists, not modify input collections

## Example Implementation

```python
class SearchFilter:
    @staticmethod
    def search_by_keyword(tasks, keyword):
        if not keyword:
            return tasks
        keyword_lower = keyword.lower()
        return [t for t in tasks if keyword_lower in t.title.lower()]

    @staticmethod
    def filter_by_status(tasks, completed):
        return [t for t in tasks if t.completed == completed]

    @staticmethod
    def filter_by_priority(tasks, priority):
        priority_norm = priority.lower()
        return [t for t in tasks if t.priority.lower() == priority_norm]

    @staticmethod
    def filter_by_tag(tasks, tag):
        tag_lower = tag.lower()
        return [t for t in tasks if any(tag_lower == t.lower() for t in t.tags)]
```

## Edge Cases

| Scenario | Handling |
|----------|----------|
| Empty task list | Return empty list without error |
| None keyword/tag | Treat as no filter (return all) |
| Non-existent priority | Return empty list |
| Tasks with missing fields | Skip or handle gracefully |
| Case variations | Normalize comparisons to lowercase |

## Quality Assurance

Before finalizing, verify:
- [ ] Method signatures match specification
- [ ] Return type is consistently a list (even if empty)
- [ ] Original task data not mutated
- [ ] Edge cases tested (empty input, None values, no matches)
- [ ] Code follows Python best practices

## Chained Operations

Support combining filters:
```python
# Example: Completed High priority tasks with 'bug' tag
results = tasks
results = SearchFilter.filter_by_status(results, True)
results = SearchFilter.filter_by_priority(results, "High")
results = SearchFilter.filter_by_tag(results, "bug")
```
