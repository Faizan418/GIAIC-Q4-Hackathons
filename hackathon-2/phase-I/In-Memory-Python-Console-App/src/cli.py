"""
Command-line interface and user interaction.
"""

import sys
from typing import Optional
from . import services
from .models import Task

# Ensure UTF-8 encoding for console output (cross-platform)
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except AttributeError:
        # Python < 3.7
        import codecs
        sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')


def display_main_menu() -> None:
    """Display the main menu options."""
    # Check and display reminders first
    display_reminders()

    print("\n" + "="*50)
    print("           TODO APPLICATION")
    print("="*50)
    print("\n[1] Add Task")
    print("[2] View All Tasks")
    print("[3] Update Task")
    print("[4] Delete Task")
    print("[5] Mark as Complete/Incomplete")
    print("[6] Search Tasks")
    print("[7] Filter Tasks")
    print("[8] Sort Tasks")
    print("[9] Set Due Date")
    print("[10] Set Recurring Task")
    print("[0] Exit")
    print("-"*50)


def get_menu_choice() -> str:
    """
    Get and validate menu choice from user.

    Returns:
        User's menu choice as string
    """
    try:
        choice = input("Enter your choice: ").strip()
        return choice
    except (KeyboardInterrupt, EOFError):
        print("\n\nExiting application...")
        return "0"


def get_user_input(prompt: str) -> str:
    """
    Get input from user with a prompt.

    Args:
        prompt: Prompt message to display

    Returns:
        User input string
    """
    return input(prompt).strip()


def confirm_action(message: str) -> bool:
    """
    Get yes/no confirmation from user.

    Args:
        message: Confirmation question

    Returns:
        True if user confirms, False otherwise
    """
    response = input(f"{message} (yes/no): ").strip().lower()
    return response in ['yes', 'y']


def display_task_list(tasks: list[Task], title: str = "Tasks") -> None:
    """
    Display a list of tasks.

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
        print(task)

    print(f"{'='*50}")


def handle_add_task() -> None:
    """Handle adding a new task."""
    print("\n--- Add New Task ---")

    try:
        title = get_user_input("Enter task title: ")

        if not title:
            print("\n✗ Error: Task title cannot be empty")
            return

        # Get priority
        print("\nPriority levels:")
        print("  [1] High")
        print("  [2] Medium (default)")
        print("  [3] Low")
        priority_choice = get_user_input("Select priority (1-3, press Enter for Medium): ")

        priority_map = {"1": "High", "2": "Medium", "3": "Low", "": "Medium"}
        priority = priority_map.get(priority_choice, "Medium")

        # Get tags
        tags_input = get_user_input("Enter tags (comma-separated, optional): ")
        tags = [tag.strip() for tag in tags_input.split(",") if tag.strip()] if tags_input else []

        task = services.create_task(title, priority, tags)
        print(f"\n✓ Task added successfully! (ID: {task.id})")
        print(task)

    except ValueError as e:
        print(f"\n✗ Error: {e}")
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")


def handle_view_tasks() -> None:
    """Handle viewing all tasks."""
    print("\n--- All Tasks ---")

    tasks = services.read_all_tasks()

    if not tasks:
        print("\nNo tasks found. Add some tasks to get started!")
        return

    display_task_list(tasks, "All Tasks")


def handle_update_task() -> None:
    """Handle updating a task."""
    print("\n--- Update Task ---")

    try:
        task_id_str = get_user_input("Enter task ID to update: ")

        if not task_id_str:
            print("\n✗ Error: Task ID cannot be empty")
            return

        try:
            task_id = int(task_id_str)
        except ValueError:
            print("\n✗ Error: Task ID must be a number")
            return

        # Check if task exists
        task = services.read_task(task_id)
        if not task:
            print(f"\n✗ Task #{task_id} not found.")
            return

        print("\nCurrent task:")
        print(task)

        new_title = get_user_input(f"\nEnter new title (press Enter to keep '{task.title}'): ")

        # Get new priority
        print("\nUpdate priority? (press Enter to skip)")
        print("  [1] High")
        print("  [2] Medium")
        print("  [3] Low")
        priority_choice = get_user_input("Select priority (1-3 or Enter to skip): ")
        priority_map = {"1": "High", "2": "Medium", "3": "Low"}
        new_priority = priority_map.get(priority_choice) if priority_choice else None

        # Get new tags
        tags_input = get_user_input("\nEnter new tags (comma-separated, or Enter to skip): ")
        new_tags = [tag.strip() for tag in tags_input.split(",") if tag.strip()] if tags_input else None

        # Check if any changes were made
        if not new_title and new_priority is None and new_tags is None:
            print("\n✗ No changes made.")
            return

        success = services.update_task(task_id, title=new_title if new_title else None,
                                      priority=new_priority, tags=new_tags)

        if success:
            print(f"\n✓ Task #{task_id} updated successfully!")
            updated_task = services.read_task(task_id)
            print(updated_task)
        else:
            print(f"\n✗ Failed to update task #{task_id}")

    except ValueError as e:
        print(f"\n✗ Error: {e}")
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")


def handle_delete_task() -> None:
    """Handle deleting a task."""
    print("\n--- Delete Task ---")

    try:
        task_id_str = get_user_input("Enter task ID to delete: ")

        if not task_id_str:
            print("\n✗ Error: Task ID cannot be empty")
            return

        try:
            task_id = int(task_id_str)
        except ValueError:
            print("\n✗ Error: Task ID must be a number")
            return

        # Check if task exists and show it
        task = services.read_task(task_id)
        if not task:
            print(f"\n✗ Task #{task_id} not found.")
            return

        print("\nTask to delete:")
        print(task)

        if confirm_action("\nAre you sure you want to delete this task?"):
            success = services.delete_task(task_id)

            if success:
                print(f"\n✓ Task #{task_id} deleted successfully!")
            else:
                print(f"\n✗ Failed to delete task #{task_id}")
        else:
            print("\nDeletion cancelled.")

    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")


def handle_toggle_completion() -> None:
    """Handle toggling task completion status."""
    print("\n--- Toggle Task Completion ---")

    try:
        task_id_str = get_user_input("Enter task ID: ")

        if not task_id_str:
            print("\n✗ Error: Task ID cannot be empty")
            return

        try:
            task_id = int(task_id_str)
        except ValueError:
            print("\n✗ Error: Task ID must be a number")
            return

        success = services.toggle_completion(task_id)

        if success:
            task = services.read_task(task_id)
            status = "completed" if task.completed else "pending"
            print(f"\n✓ Task #{task_id} marked as {status}!")
            print(task)
        else:
            print(f"\n✗ Task #{task_id} not found.")

    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")


def handle_search() -> None:
    """Handle searching tasks by keyword."""
    print("\n--- Search Tasks ---")

    keyword = get_user_input("Enter search keyword: ")

    if not keyword:
        print("\n✗ Search keyword cannot be empty")
        return

    results = services.search_tasks(keyword)

    if results:
        display_task_list(results, f"Search Results for '{keyword}'")
    else:
        print(f"\nNo tasks found matching '{keyword}'")


def handle_filter() -> None:
    """Handle filtering tasks."""
    print("\n--- Filter Tasks ---")
    print("[1] Show Completed Tasks")
    print("[2] Show Pending Tasks")
    print("[3] Show High Priority")
    print("[4] Show Medium Priority")
    print("[5] Show Low Priority")
    print("[6] Show Tasks by Tag")
    print("[0] Back to Main Menu")

    choice = get_user_input("\nSelect filter: ")

    try:
        if choice == "1":
            tasks = services.filter_by_status(completed=True)
            display_task_list(tasks, "Completed Tasks")
        elif choice == "2":
            tasks = services.filter_by_status(completed=False)
            display_task_list(tasks, "Pending Tasks")
        elif choice == "3":
            tasks = services.filter_by_priority("High")
            display_task_list(tasks, "High Priority Tasks")
        elif choice == "4":
            tasks = services.filter_by_priority("Medium")
            display_task_list(tasks, "Medium Priority Tasks")
        elif choice == "5":
            tasks = services.filter_by_priority("Low")
            display_task_list(tasks, "Low Priority Tasks")
        elif choice == "6":
            tag = get_user_input("Enter tag name: ")
            if tag:
                tasks = services.filter_by_tag(tag)
                display_task_list(tasks, f"Tasks with tag '{tag}'")
        elif choice == "0":
            return
        else:
            print(f"\n✗ Invalid choice '{choice}'")

    except ValueError as e:
        print(f"\n✗ Error: {e}")


def handle_sort() -> None:
    """Handle sorting tasks."""
    print("\n--- Sort Tasks ---")
    print("[1] Sort by Priority (High to Low)")
    print("[2] Sort by Date (Newest First)")
    print("[3] Sort by Date (Oldest First)")
    print("[4] Sort by Title (A-Z)")
    print("[5] Sort by Title (Z-A)")
    print("[0] Back to Main Menu")

    choice = get_user_input("\nSelect sort option: ")

    tasks = services.read_all_tasks()

    if not tasks:
        print("\nNo tasks to sort.")
        return

    try:
        if choice == "1":
            sorted_tasks = services.sort_tasks(tasks, by="priority", reverse=True)
            display_task_list(sorted_tasks, "Tasks by Priority")
        elif choice == "2":
            sorted_tasks = services.sort_tasks(tasks, by="date", reverse=True)
            display_task_list(sorted_tasks, "Tasks (Newest First)")
        elif choice == "3":
            sorted_tasks = services.sort_tasks(tasks, by="date", reverse=False)
            display_task_list(sorted_tasks, "Tasks (Oldest First)")
        elif choice == "4":
            sorted_tasks = services.sort_tasks(tasks, by="title", reverse=False)
            display_task_list(sorted_tasks, "Tasks (A-Z)")
        elif choice == "5":
            sorted_tasks = services.sort_tasks(tasks, by="title", reverse=True)
            display_task_list(sorted_tasks, "Tasks (Z-A)")
        elif choice == "0":
            return
        else:
            print(f"\n✗ Invalid choice '{choice}'")

    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")


def display_reminders() -> None:
    """Display reminder notifications for tasks due now."""
    reminders = services.check_reminders()
    if reminders:
        print("\n" + "="*50)
        print("⏰ REMINDERS - Tasks Due Now!")
        print("="*50)
        for task in reminders:
            print(f"  {task}")
        print("="*50)


def handle_set_due_date() -> None:
    """Handle setting due date for a task."""
    print("\n--- Set Due Date ---")

    try:
        task_id_str = get_user_input("Enter task ID: ")

        if not task_id_str:
            print("\n✗ Error: Task ID cannot be empty")
            return

        try:
            task_id = int(task_id_str)
        except ValueError:
            print("\n✗ Error: Task ID must be a number")
            return

        task = services.read_task(task_id)
        if not task:
            print(f"\n✗ Task #{task_id} not found.")
            return

        print(f"\nCurrent task: {task}")

        date_str = get_user_input("\nEnter due date (YYYY-MM-DD HH:MM or YYYY-MM-DD): ")

        if not date_str:
            print("\n✗ No due date entered.")
            return

        due_date = services.validate_date(date_str)
        if not due_date:
            print("\n✗ Invalid date format. Use: YYYY-MM-DD HH:MM or YYYY-MM-DD")
            return

        success = services.set_due_date(task_id, due_date)

        if success:
            print(f"\n✓ Due date set for task #{task_id}!")
            updated_task = services.read_task(task_id)
            print(updated_task)
        else:
            print(f"\n✗ Failed to set due date for task #{task_id}")

    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")


def handle_set_recurrence() -> None:
    """Handle setting recurrence for a task."""
    print("\n--- Set Recurring Task ---")

    try:
        task_id_str = get_user_input("Enter task ID: ")

        if not task_id_str:
            print("\n✗ Error: Task ID cannot be empty")
            return

        try:
            task_id = int(task_id_str)
        except ValueError:
            print("\n✗ Error: Task ID must be a number")
            return

        task = services.read_task(task_id)
        if not task:
            print(f"\n✗ Task #{task_id} not found.")
            return

        print(f"\nCurrent task: {task}")

        print("\nRecurrence frequency:")
        print("  [1] Daily")
        print("  [2] Weekly")
        print("  [3] Monthly")

        freq_choice = get_user_input("Select frequency (1-3): ")
        freq_map = {"1": "Daily", "2": "Weekly", "3": "Monthly"}
        frequency = freq_map.get(freq_choice)

        if not frequency:
            print("\n✗ Invalid frequency selection.")
            return

        date_str = get_user_input("\nEnter start date (YYYY-MM-DD HH:MM or YYYY-MM-DD): ")

        if not date_str:
            print("\n✗ No start date entered.")
            return

        start_date = services.validate_date(date_str)
        if not start_date:
            print("\n✗ Invalid date format. Use: YYYY-MM-DD HH:MM or YYYY-MM-DD")
            return

        success = services.set_recurrence(task_id, frequency, start_date)

        if success:
            print(f"\n✓ Recurrence set for task #{task_id} ({frequency})!")
            print("Note: Task will regenerate automatically when marked complete.")
            updated_task = services.read_task(task_id)
            print(updated_task)
        else:
            print(f"\n✗ Failed to set recurrence for task #{task_id}")

    except ValueError as e:
        print(f"\n✗ Error: {e}")
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")


def run_app() -> None:
    """Main application loop."""
    print("\n✓ Todo Application Started!")

    while True:
        display_main_menu()
        choice = get_menu_choice()

        if choice == "1":
            handle_add_task()
        elif choice == "2":
            handle_view_tasks()
        elif choice == "3":
            handle_update_task()
        elif choice == "4":
            handle_delete_task()
        elif choice == "5":
            handle_toggle_completion()
        elif choice == "6":
            handle_search()
        elif choice == "7":
            handle_filter()
        elif choice == "8":
            handle_sort()
        elif choice == "9":
            handle_set_due_date()
        elif choice == "10":
            handle_set_recurrence()
        elif choice == "0":
            print("\nThank you for using Todo Application!")
            print("Goodbye!\n")
            break
        else:
            print(f"\n✗ Invalid choice '{choice}'. Please select 0-10.")

        input("\nPress Enter to continue...")
