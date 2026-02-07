---
name: python-todo-organization
description: Guide for organizing Todo tasks using priorities, tags, search, and filtering. This skill should be used when improving task usability and organization.
license: MIT
---

# Python Todo Organization Skill

Enhance task discoverability and productivity through advanced organization features.

## Purpose
Add priority management, tag-based organization, search functionality, and filtering capabilities to help users find and organize tasks efficiently.

## When to Use
- Implementing search functionality
- Adding filter capabilities
- Implementing priority-based views
- Creating tag management features
- Building sorting mechanisms

## Core Responsibilities
1. **Search** - Find tasks by keyword
2. **Filter** - Show tasks by status, priority, or tags
3. **Sort** - Order tasks by different criteria
4. **Priority Management** - Organize by importance
5. **Tag System** - Categorize tasks flexibly

## Implementation Guidelines

### 1. Search Functions
```python
def search_tasks_by_keyword(keyword: str) -> list:
    """
    Search tasks by keyword in title

    Args:
        keyword: Search term (case-insensitive)

    Returns:
        List of matching tasks
    """
    if not keyword or not keyword.strip():
        return []

    keyword_lower = keyword.strip().lower()
    from core import get_all_tasks

    matching_tasks = [
        task for task in get_all_tasks()
        if keyword_lower in task.title.lower()
    ]

    return matching_tasks

def search_tasks_by_tag(tag: str) -> list:
    """
    Find all tasks with a specific tag

    Args:
        tag: Tag name to search for

    Returns:
        List of tasks containing the tag
    """
    if not tag or not tag.strip():
        return []

    tag_lower = tag.strip().lower()
    from core import get_all_tasks

    matching_tasks = [
        task for task in get_all_tasks()
        if any(t.lower() == tag_lower for t in task.tags)
    ]

    return matching_tasks

def search_tasks_advanced(keyword: str = None, tags: list = None,
                          priority: str = None, completed: bool = None) -> list:
    """
    Advanced search with multiple criteria

    Args:
        keyword: Search term for title
        tags: List of tags to match (any)
        priority: Priority level to filter
        completed: Completion status filter

    Returns:
        List of tasks matching all specified criteria
    """
    from core import get_all_tasks
    results = get_all_tasks()

    # Filter by keyword
    if keyword and keyword.strip():
        keyword_lower = keyword.strip().lower()
        results = [t for t in results if keyword_lower in t.title.lower()]

    # Filter by tags
    if tags:
        tags_lower = [tag.lower() for tag in tags]
        results = [
            t for t in results
            if any(tag.lower() in tags_lower for tag in t.tags)
        ]

    # Filter by priority
    if priority:
        results = [t for t in results if t.priority == priority]

    # Filter by completion status
    if completed is not None:
        results = [t for t in results if t.completed == completed]

    return results
```

### 2. Filter Functions
```python
def filter_by_status(completed: bool = False) -> list:
    """
    Filter tasks by completion status

    Args:
        completed: True for completed tasks, False for pending

    Returns:
        List of filtered tasks
    """
    from core import get_all_tasks
    return [task for task in get_all_tasks() if task.completed == completed]

def filter_by_priority(priority: str) -> list:
    """
    Filter tasks by priority level

    Args:
        priority: Priority level (High/Medium/Low)

    Returns:
        List of tasks with matching priority
    """
    valid_priorities = ["High", "Medium", "Low"]
    if priority not in valid_priorities:
        raise ValueError(f"Priority must be one of {valid_priorities}")

    from core import get_all_tasks
    return [task for task in get_all_tasks() if task.priority == priority]

def filter_by_multiple_tags(tags: list, match_all: bool = False) -> list:
    """
    Filter tasks by multiple tags

    Args:
        tags: List of tag names
        match_all: If True, task must have all tags; if False, any tag

    Returns:
        List of matching tasks
    """
    if not tags:
        return []

    tags_lower = [tag.lower() for tag in tags]
    from core import get_all_tasks

    if match_all:
        # Task must have all specified tags
        matching_tasks = [
            task for task in get_all_tasks()
            if all(any(t.lower() == tag_lower for t in task.tags)
                   for tag_lower in tags_lower)
        ]
    else:
        # Task must have at least one specified tag
        matching_tasks = [
            task for task in get_all_tasks()
            if any(any(t.lower() == tag_lower for t in task.tags)
                   for tag_lower in tags_lower)
        ]

    return matching_tasks

def get_tasks_without_tags() -> list:
    """Get all tasks that have no tags"""
    from core import get_all_tasks
    return [task for task in get_all_tasks() if not task.tags]
```

### 3. Sort Functions
```python
def sort_tasks_by_priority(tasks: list = None, reverse: bool = True) -> list:
    """
    Sort tasks by priority (High > Medium > Low)

    Args:
        tasks: List of tasks to sort (or all if None)
        reverse: True for High first, False for Low first

    Returns:
        Sorted list of tasks
    """
    if tasks is None:
        from core import get_all_tasks
        tasks = get_all_tasks()

    priority_order = {"High": 3, "Medium": 2, "Low": 1}

    return sorted(
        tasks,
        key=lambda t: priority_order.get(t.priority, 0),
        reverse=reverse
    )

def sort_tasks_by_date(tasks: list = None, reverse: bool = True) -> list:
    """
    Sort tasks by creation date

    Args:
        tasks: List of tasks to sort (or all if None)
        reverse: True for newest first, False for oldest first

    Returns:
        Sorted list of tasks
    """
    if tasks is None:
        from core import get_all_tasks
        tasks = get_all_tasks()

    return sorted(tasks, key=lambda t: t.created_at, reverse=reverse)

def sort_tasks_by_title(tasks: list = None, reverse: bool = False) -> list:
    """
    Sort tasks alphabetically by title

    Args:
        tasks: List of tasks to sort (or all if None)
        reverse: True for Z-A, False for A-Z

    Returns:
        Sorted list of tasks
    """
    if tasks is None:
        from core import get_all_tasks
        tasks = get_all_tasks()

    return sorted(tasks, key=lambda t: t.title.lower(), reverse=reverse)

def sort_tasks_by_completion(tasks: list = None) -> list:
    """
    Sort tasks with pending first, completed last

    Args:
        tasks: List of tasks to sort (or all if None)

    Returns:
        Sorted list (pending first)
    """
    if tasks is None:
        from core import get_all_tasks
        tasks = get_all_tasks()

    return sorted(tasks, key=lambda t: t.completed)
```

### 4. Tag Management
```python
def get_all_tags() -> list:
    """
    Get list of all unique tags used across tasks

    Returns:
        Sorted list of unique tag names
    """
    from core import get_all_tasks
    all_tags = set()

    for task in get_all_tasks():
        all_tags.update(task.tags)

    return sorted(all_tags)

def get_tag_statistics() -> dict:
    """
    Get usage count for each tag

    Returns:
        Dictionary mapping tag names to usage counts
    """
    from core import get_all_tasks
    tag_counts = {}

    for task in get_all_tasks():
        for tag in task.tags:
            tag_counts[tag] = tag_counts.get(tag, 0) + 1

    return dict(sorted(tag_counts.items(), key=lambda x: x[1], reverse=True))

def add_tag_to_task(task_id: int, tag: str) -> bool:
    """
    Add a tag to an existing task

    Args:
        task_id: Task ID
        tag: Tag name to add

    Returns:
        True if tag added, False if task not found
    """
    from core import get_task_by_id

    task = get_task_by_id(task_id)
    if not task:
        return False

    tag_cleaned = tag.strip()
    if not tag_cleaned:
        raise ValueError("Tag cannot be empty")

    if tag_cleaned not in task.tags:
        task.tags.append(tag_cleaned)

    return True

def remove_tag_from_task(task_id: int, tag: str) -> bool:
    """
    Remove a tag from a task

    Args:
        task_id: Task ID
        tag: Tag name to remove

    Returns:
        True if tag removed, False if task not found or tag not present
    """
    from core import get_task_by_id

    task = get_task_by_id(task_id)
    if not task:
        return False

    tag_cleaned = tag.strip()
    if tag_cleaned in task.tags:
        task.tags.remove(tag_cleaned)
        return True

    return False
```

### 5. Priority Management
```python
def get_priority_statistics() -> dict:
    """
    Get count of tasks by priority level

    Returns:
        Dictionary with counts for each priority
    """
    from core import get_all_tasks

    stats = {"High": 0, "Medium": 0, "Low": 0}

    for task in get_all_tasks():
        if task.priority in stats:
            stats[task.priority] += 1

    return stats

def get_high_priority_pending() -> list:
    """
    Get all high priority tasks that are not completed

    Returns:
        List of high priority pending tasks
    """
    from core import get_all_tasks

    return [
        task for task in get_all_tasks()
        if task.priority == "High" and not task.completed
    ]
```

### 6. CLI Handlers for Organization
```python
def handle_search_tasks():
    """Handle searching tasks by keyword"""
    print("\n--- Search Tasks ---")

    keyword = input("Enter search keyword: ").strip()

    if not keyword:
        print("\n✗ Search keyword cannot be empty")
        return

    results = search_tasks_by_keyword(keyword)

    if results:
        from cli import display_task_list
        display_task_list(results, f"Search Results for '{keyword}'")
    else:
        print(f"\nNo tasks found matching '{keyword}'")

def handle_filter_menu():
    """Display and handle filter menu"""
    print("\n--- Filter Tasks ---")
    print("[1] Show Completed Tasks")
    print("[2] Show Pending Tasks")
    print("[3] Show High Priority")
    print("[4] Show Medium Priority")
    print("[5] Show Low Priority")
    print("[6] Show Tasks by Tag")
    print("[0] Back to Main Menu")

    choice = input("\nSelect filter: ").strip()

    if choice == "1":
        tasks = filter_by_status(completed=True)
        from cli import display_task_list
        display_task_list(tasks, "Completed Tasks")

    elif choice == "2":
        tasks = filter_by_status(completed=False)
        from cli import display_task_list
        display_task_list(tasks, "Pending Tasks")

    elif choice in ["3", "4", "5"]:
        priority_map = {"3": "High", "4": "Medium", "5": "Low"}
        priority = priority_map[choice]
        tasks = filter_by_priority(priority)
        from cli import display_task_list
        display_task_list(tasks, f"{priority} Priority Tasks")

    elif choice == "6":
        handle_filter_by_tag()

    elif choice == "0":
        return

    else:
        print(f"\n✗ Invalid choice '{choice}'")

def handle_filter_by_tag():
    """Handle filtering by tag"""
    # Show available tags
    tags = get_all_tags()

    if not tags:
        print("\nNo tags found in any tasks.")
        return

    print("\nAvailable tags:")
    for i, tag in enumerate(tags, 1):
        print(f"  [{i}] {tag}")

    tag_input = input("\nEnter tag name: ").strip()

    if not tag_input:
        return

    results = search_tasks_by_tag(tag_input)
    from cli import display_task_list
    display_task_list(results, f"Tasks with tag '{tag_input}'")

def handle_sort_menu():
    """Display and handle sort menu"""
    print("\n--- Sort Tasks ---")
    print("[1] Sort by Priority (High to Low)")
    print("[2] Sort by Date (Newest First)")
    print("[3] Sort by Date (Oldest First)")
    print("[4] Sort by Title (A-Z)")
    print("[5] Sort by Completion Status")
    print("[0] Back to Main Menu")

    choice = input("\nSelect sort option: ").strip()

    from core import get_all_tasks
    tasks = get_all_tasks()

    if choice == "1":
        sorted_tasks = sort_tasks_by_priority(tasks)
        from cli import display_task_list
        display_task_list(sorted_tasks, "Tasks by Priority")

    elif choice == "2":
        sorted_tasks = sort_tasks_by_date(tasks, reverse=True)
        from cli import display_task_list
        display_task_list(sorted_tasks, "Tasks (Newest First)")

    elif choice == "3":
        sorted_tasks = sort_tasks_by_date(tasks, reverse=False)
        from cli import display_task_list
        display_task_list(sorted_tasks, "Tasks (Oldest First)")

    elif choice == "4":
        sorted_tasks = sort_tasks_by_title(tasks)
        from cli import display_task_list
        display_task_list(sorted_tasks, "Tasks (Alphabetical)")

    elif choice == "5":
        sorted_tasks = sort_tasks_by_completion(tasks)
        from cli import display_task_list
        display_task_list(sorted_tasks, "Tasks (Pending First)")

    elif choice == "0":
        return

    else:
        print(f"\n✗ Invalid choice '{choice}'")

def handle_view_all_tags():
    """Display all tags with usage statistics"""
    print("\n--- All Tags ---")

    tag_stats = get_tag_statistics()

    if not tag_stats:
        print("\nNo tags found.")
        return

    print(f"\nTotal unique tags: {len(tag_stats)}\n")
    print(f"{'Tag':<20} {'Usage Count':<10}")
    print("-" * 30)

    for tag, count in tag_stats.items():
        print(f"{tag:<20} {count:<10}")
```

## Key Principles
- **Performance** - Efficient filtering and searching
- **Flexibility** - Multiple ways to find tasks
- **User Control** - Let users choose organization method
- **Non-Destructive** - Filters don't modify data
- **Composable** - Combine filters and sorts

## Best Practices
- Return copies, never modify original lists
- Handle empty results gracefully
- Case-insensitive searches
- Provide multiple sort/filter options
- Show result counts to users

## Anti-Patterns to Avoid
- Modifying task list during filtering
- Case-sensitive searches
- Hardcoded filter values
- No validation of filter criteria
- Ignoring empty result sets

## Integration Points
```python
# Main menu updates
elif choice == "6":
    handle_filter_menu()
elif choice == "7":
    handle_search_tasks()
elif choice == "9":
    handle_sort_menu()
elif choice == "10":
    handle_view_all_tags()
```

## Next Steps
Once organization features are implemented:
1. Add advanced search combinations
2. Implement saved filters
3. Create smart lists (e.g., "Today's priorities")
4. Add comprehensive tests (python-todo-testing skill)
