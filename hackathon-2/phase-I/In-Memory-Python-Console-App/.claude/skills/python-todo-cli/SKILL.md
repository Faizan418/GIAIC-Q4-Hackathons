---
name: python-todo-cli
description: Guide for building a clean command-line interface for a Python Todo application. This skill should be used when designing menus and handling user input.
license: MIT
---

# Python Todo CLI Skill

Create a professional, user-friendly console interface for the Todo application.

## Purpose
Design intuitive menus, handle user input safely, and provide clear feedback through the command-line interface.

## When to Use
- Implementing the main menu loop
- Handling user input validation
- Displaying task lists and details
- Creating interactive prompts
- Formatting console output

## Core Responsibilities
1. **Menu Display** - Show clear, numbered menu options
2. **Input Validation** - Safely handle user input
3. **Output Formatting** - Present tasks in readable format
4. **Error Messaging** - Display helpful error messages
5. **User Feedback** - Confirm actions and show results

## Implementation Guidelines

### 1. Main Menu Structure
```python
def display_main_menu():
    """Display the main menu options"""
    print("\n" + "="*50)
    print("           TODO APPLICATION")
    print("="*50)
    print("\n[1] View All Tasks")
    print("[2] Add New Task")
    print("[3] Update Task")
    print("[4] Delete Task")
    print("[5] Toggle Task Completion")
    print("[6] Filter Tasks")
    print("[7] Search Tasks")
    print("[8] View Statistics")
    print("[0] Exit")
    print("-"*50)
```

### 2. Input Handling
```python
def get_menu_choice() -> str:
    """
    Get and validate menu choice

    Returns:
        User's menu choice as string
    """
    try:
        choice = input("Enter your choice: ").strip()
        return choice
    except (KeyboardInterrupt, EOFError):
        print("\n\nExiting application...")
        return "0"

def get_task_id() -> int:
    """
    Prompt for and validate task ID

    Returns:
        Valid task ID

    Raises:
        ValueError: If input is not a valid integer
    """
    task_id_str = input("Enter task ID: ").strip()
    if not task_id_str:
        raise ValueError("Task ID cannot be empty")

    try:
        task_id = int(task_id_str)
        if task_id <= 0:
            raise ValueError("Task ID must be positive")
        return task_id
    except ValueError:
        raise ValueError("Task ID must be a valid number")

def get_task_title() -> str:
    """
    Prompt for task title with validation

    Returns:
        Non-empty task title
    """
    title = input("Enter task title: ").strip()
    if not title:
        raise ValueError("Task title cannot be empty")
    return title

def get_priority() -> str:
    """
    Prompt for priority level

    Returns:
        Valid priority (High/Medium/Low)
    """
    print("\nPriority levels:")
    print("  [1] High")
    print("  [2] Medium")
    print("  [3] Low")

    choice = input("Select priority (1-3, default=2): ").strip()

    priority_map = {
        "1": "High",
        "2": "Medium",
        "3": "Low",
        "": "Medium"  # Default
    }

    if choice in priority_map:
        return priority_map[choice]
    else:
        print("Invalid choice, defaulting to Medium")
        return "Medium"

def get_tags() -> list:
    """
    Prompt for task tags

    Returns:
        List of tags
    """
    tags_input = input("Enter tags (comma-separated, optional): ").strip()
    if not tags_input:
        return []

    tags = [tag.strip() for tag in tags_input.split(",") if tag.strip()]
    return tags
```

### 3. Display Functions
```python
def display_task(task):
    """
    Display a single task with formatting

    Args:
        task: Task object to display
    """
    status = "✓" if task.completed else " "
    priority_icon = {"High": "🔴", "Medium": "🟡", "Low": "🟢"}

    print(f"\n[{status}] Task #{task.id}: {task.title}")
    print(f"    Priority: {priority_icon.get(task.priority, '')} {task.priority}")

    if task.tags:
        print(f"    Tags: {', '.join(task.tags)}")

    print(f"    Created: {task.created_at.strftime('%Y-%m-%d %H:%M')}")

def display_task_list(tasks, title="Tasks"):
    """
    Display a list of tasks

    Args:
        tasks: List of Task objects
        title: Header title for the list
    """
    if not tasks:
        print(f"\nNo {title.lower()} found.")
        return

    print(f"\n{'='*50}")
    print(f"  {title} ({len(tasks)} total)")
    print(f"{'='*50}")

    for task in tasks:
        display_task(task)

    print(f"{'='*50}")

def display_task_table(tasks):
    """
    Display tasks in table format

    Args:
        tasks: List of Task objects
    """
    if not tasks:
        print("\nNo tasks to display.")
        return

    # Header
    print(f"\n{'ID':<5} {'✓':<3} {'Title':<30} {'Priority':<10} {'Tags':<20}")
    print("-" * 70)

    # Rows
    for task in tasks:
        status = "✓" if task.completed else " "
        title_short = task.title[:27] + "..." if len(task.title) > 30 else task.title
        tags_str = ", ".join(task.tags[:2])
        if len(task.tags) > 2:
            tags_str += "..."

        print(f"{task.id:<5} {status:<3} {title_short:<30} {task.priority:<10} {tags_str:<20}")

    print("-" * 70)
```

### 4. Menu Action Handlers
```python
def handle_add_task():
    """Handle adding a new task"""
    print("\n--- Add New Task ---")

    try:
        title = get_task_title()
        priority = get_priority()
        tags = get_tags()

        # Call core function
        from core import add_task
        task = add_task(title, priority, tags)

        print(f"\n✓ Task added successfully! (ID: {task.id})")
        display_task(task)

    except ValueError as e:
        print(f"\n✗ Error: {e}")
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")

def handle_view_all_tasks():
    """Handle viewing all tasks"""
    print("\n--- All Tasks ---")

    from core import get_all_tasks
    tasks = get_all_tasks()

    if not tasks:
        print("\nNo tasks found. Add some tasks to get started!")
        return

    display_task_table(tasks)

def handle_update_task():
    """Handle updating a task"""
    print("\n--- Update Task ---")

    try:
        task_id = get_task_id()

        # Check if task exists
        from core import get_task_by_id
        task = get_task_by_id(task_id)

        if not task:
            print(f"\n✗ Task #{task_id} not found.")
            return

        print("\nCurrent task:")
        display_task(task)

        print("\nEnter new values (press Enter to keep current):")

        # Get new title
        new_title_input = input(f"New title [{task.title}]: ").strip()
        new_title = new_title_input if new_title_input else None

        # Get new priority
        print("\nUpdate priority?")
        new_priority = None
        update_priority = input("Change priority? (y/n): ").strip().lower()
        if update_priority == 'y':
            new_priority = get_priority()

        # Update the task
        from core import update_task
        success = update_task(task_id, title=new_title, priority=new_priority)

        if success:
            print(f"\n✓ Task #{task_id} updated successfully!")
        else:
            print(f"\n✗ Failed to update task #{task_id}")

    except ValueError as e:
        print(f"\n✗ Error: {e}")

def handle_delete_task():
    """Handle deleting a task"""
    print("\n--- Delete Task ---")

    try:
        task_id = get_task_id()

        # Show task before deleting
        from core import get_task_by_id
        task = get_task_by_id(task_id)

        if not task:
            print(f"\n✗ Task #{task_id} not found.")
            return

        print("\nTask to delete:")
        display_task(task)

        confirm = input("\nAre you sure you want to delete this task? (yes/no): ").strip().lower()

        if confirm in ['yes', 'y']:
            from core import delete_task
            success = delete_task(task_id)

            if success:
                print(f"\n✓ Task #{task_id} deleted successfully!")
            else:
                print(f"\n✗ Failed to delete task #{task_id}")
        else:
            print("\nDeletion cancelled.")

    except ValueError as e:
        print(f"\n✗ Error: {e}")

def handle_toggle_completion():
    """Handle toggling task completion status"""
    print("\n--- Toggle Task Completion ---")

    try:
        task_id = get_task_id()

        from core import toggle_task_completion
        success = toggle_task_completion(task_id)

        if success:
            from core import get_task_by_id
            task = get_task_by_id(task_id)
            status = "completed" if task.completed else "pending"
            print(f"\n✓ Task #{task_id} marked as {status}!")
        else:
            print(f"\n✗ Task #{task_id} not found.")

    except ValueError as e:
        print(f"\n✗ Error: {e}")

def handle_view_statistics():
    """Handle viewing task statistics"""
    print("\n--- Task Statistics ---")

    from core import get_task_count
    stats = get_task_count()

    print(f"\nTotal Tasks:     {stats['total']}")
    print(f"Completed:       {stats['completed']}")
    print(f"Pending:         {stats['pending']}")

    if stats['total'] > 0:
        completion_rate = (stats['completed'] / stats['total']) * 100
        print(f"Completion Rate: {completion_rate:.1f}%")
```

### 5. Main Application Loop
```python
def run_app():
    """Main application loop"""
    print("\n✓ Todo Application Started!")

    while True:
        display_main_menu()

        choice = get_menu_choice()

        if choice == "1":
            handle_view_all_tasks()
        elif choice == "2":
            handle_add_task()
        elif choice == "3":
            handle_update_task()
        elif choice == "4":
            handle_delete_task()
        elif choice == "5":
            handle_toggle_completion()
        elif choice == "6":
            # Will be implemented with organization skill
            print("\n[Filter feature - coming soon]")
        elif choice == "7":
            # Will be implemented with organization skill
            print("\n[Search feature - coming soon]")
        elif choice == "8":
            handle_view_statistics()
        elif choice == "0":
            print("\nThank you for using Todo Application!")
            print("Goodbye!\n")
            break
        else:
            print(f"\n✗ Invalid choice '{choice}'. Please select 0-8.")

        # Pause before showing menu again
        input("\nPress Enter to continue...")

if __name__ == "__main__":
    run_app()
```

### 6. Enhanced Features
```python
def clear_screen():
    """Clear console screen (cross-platform)"""
    import os
    os.system('cls' if os.name == 'nt' else 'clear')

def display_banner():
    """Display application banner"""
    banner = """
    ╔══════════════════════════════════════╗
    ║                                      ║
    ║        TODO APPLICATION              ║
    ║        Manage Your Tasks             ║
    ║                                      ║
    ╚══════════════════════════════════════╝
    """
    print(banner)

def confirm_action(prompt: str) -> bool:
    """
    Get yes/no confirmation from user

    Args:
        prompt: Confirmation question

    Returns:
        True if confirmed, False otherwise
    """
    response = input(f"{prompt} (y/n): ").strip().lower()
    return response in ['y', 'yes']
```

## Key Principles
- **User-Friendly** - Clear prompts and helpful messages
- **Error Handling** - Graceful handling of invalid input
- **Consistent Format** - Uniform display patterns
- **Immediate Feedback** - Confirm actions immediately
- **Safe Input** - Validate before passing to core logic

## Best Practices
- Always validate input before calling core functions
- Provide default values where sensible
- Display current state before modifications
- Confirm destructive actions (delete)
- Use try-except for error handling
- Keep menu handlers focused and simple

## Anti-Patterns to Avoid
- No input validation
- Cryptic error messages
- Mixing business logic with UI code
- No confirmation for destructive actions
- Unclear menu options

## Integration with Core
```python
# Import core functions
from core import (
    add_task,
    get_all_tasks,
    update_task,
    delete_task,
    toggle_task_completion,
    get_task_by_id,
    get_task_count
)
```

## Next Steps
Once CLI is implemented:
1. Add filter/search handlers (python-todo-organization skill)
2. Enhance display with colors using colorama
3. Add keyboard shortcuts
4. Implement comprehensive tests (python-todo-testing skill)
