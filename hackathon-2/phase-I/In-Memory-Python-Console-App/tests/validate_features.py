"""
Feature Validation Script for Todo Application

Tests all 10 required features (5 Basic + 3 Intermediate + 2 Advanced)
"""

import sys
import os
from datetime import datetime, timedelta

# Fix UTF-8 encoding for Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except AttributeError:
        import codecs
        sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src import services, storage
from src.models import Task


def setup():
    """Clear storage before each test group."""
    storage.clear_storage()


def test_feature_1_add_task():
    """Feature 1: Add Task - Create new todo items"""
    print("\n[1] Testing: Add Task")

    try:
        setup()

        # Test basic task creation
        task1 = services.create_task("Buy groceries")
        assert task1.id == 1, "First task should have ID 1"
        assert task1.title == "Buy groceries", "Title should match"
        assert task1.completed == False, "New task should not be completed"

        # Test with priority and tags
        task2 = services.create_task("Meeting", priority="High", tags=["work", "urgent"])
        assert task2.priority == "High", "Priority should be set"
        assert "work" in task2.tags, "Tags should be set"

        # Test empty title validation
        try:
            services.create_task("")
            assert False, "Empty title should raise ValueError"
        except ValueError:
            pass  # Expected

        print("  ✓ Add Task: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ Add Task: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ Add Task: ERROR - {e}")
        return False


def test_feature_2_delete_task():
    """Feature 2: Delete Task - Remove tasks from the list"""
    print("\n[2] Testing: Delete Task")

    try:
        setup()

        task = services.create_task("Delete me")
        task_id = task.id

        # Delete existing task
        result = services.delete_task(task_id)
        assert result == True, "Delete should return True"

        # Verify deletion
        deleted = services.read_task(task_id)
        assert deleted is None, "Task should be deleted"

        # Test delete non-existent task
        result = services.delete_task(999)
        assert result == False, "Deleting non-existent task should return False"

        print("  ✓ Delete Task: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ Delete Task: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ Delete Task: ERROR - {e}")
        return False


def test_feature_3_update_task():
    """Feature 3: Update Task - Modify existing task details"""
    print("\n[3] Testing: Update Task")

    try:
        setup()

        task = services.create_task("Original title", priority="Low")

        # Update title
        result = services.update_task(task.id, title="Updated title")
        assert result == True, "Update should return True"
        assert task.title == "Updated title", "Title should be updated"

        # Update priority
        result = services.update_task(task.id, priority="High")
        assert task.priority == "High", "Priority should be updated"

        # Update tags
        result = services.update_task(task.id, tags=["work", "important"])
        assert "work" in task.tags, "Tags should be updated"

        # Test update non-existent task
        result = services.update_task(999, title="Test")
        assert result == False, "Updating non-existent task should return False"

        print("  ✓ Update Task: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ Update Task: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ Update Task: ERROR - {e}")
        return False


def test_feature_4_view_task_list():
    """Feature 4: View Task List - Display all tasks"""
    print("\n[4] Testing: View Task List")

    try:
        setup()

        # Test empty list
        tasks = services.read_all_tasks()
        assert len(tasks) == 0, "Should start with empty list"

        # Add tasks and view
        services.create_task("Task 1")
        services.create_task("Task 2")
        services.create_task("Task 3")

        tasks = services.read_all_tasks()
        assert len(tasks) == 3, "Should have 3 tasks"

        # Test individual task retrieval
        task = services.read_task(1)
        assert task is not None, "Should retrieve task by ID"
        assert task.id == 1, "Should get correct task"

        print("  ✓ View Task List: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ View Task List: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ View Task List: ERROR - {e}")
        return False


def test_feature_5_mark_complete():
    """Feature 5: Mark as Complete - Toggle task completion status"""
    print("\n[5] Testing: Mark as Complete")

    try:
        setup()

        task = services.create_task("Complete me")

        # Toggle to complete
        result = services.toggle_completion(task.id)
        assert result == True, "Toggle should return True"
        assert task.completed == True, "Task should be completed"

        # Toggle back to incomplete
        result = services.toggle_completion(task.id)
        assert task.completed == False, "Task should be incomplete"

        # Test toggle non-existent task
        result = services.toggle_completion(999)
        assert result == False, "Toggling non-existent task should return False"

        print("  ✓ Mark as Complete: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ Mark as Complete: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ Mark as Complete: ERROR - {e}")
        return False


def test_feature_6_priorities_and_tags():
    """Feature 6: Priorities & Tags - Assign levels and categories"""
    print("\n[6] Testing: Priorities & Tags")

    try:
        setup()

        # Test priority assignment
        task1 = services.create_task("High priority task", priority="High")
        assert task1.priority == "High", "Priority should be High"

        task2 = services.create_task("Low priority task", priority="Low")
        assert task2.priority == "Low", "Priority should be Low"

        # Test default priority
        task3 = services.create_task("Default task")
        assert task3.priority == "Medium", "Default priority should be Medium"

        # Test tag assignment
        task4 = services.create_task("Tagged task", tags=["work", "urgent", "meeting"])
        assert len(task4.tags) == 3, "Should have 3 tags"
        assert "work" in task4.tags, "Should contain work tag"

        # Test priority validation
        try:
            services.create_task("Bad priority", priority="Critical")
            assert False, "Invalid priority should raise ValueError"
        except ValueError:
            pass  # Expected

        print("  ✓ Priorities & Tags: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ Priorities & Tags: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ Priorities & Tags: ERROR - {e}")
        return False


def test_feature_7_search_and_filter():
    """Feature 7: Search & Filter - Search by keyword, filter by status/priority/tag"""
    print("\n[7] Testing: Search & Filter")

    try:
        setup()

        # Create test tasks
        services.create_task("Buy groceries", priority="High", tags=["shopping"])
        services.create_task("Buy books", priority="Low", tags=["shopping"])
        services.create_task("Read documentation", priority="Medium", tags=["work"])
        task4 = services.create_task("Complete report", priority="High", tags=["work"])
        services.toggle_completion(task4.id)

        # Test keyword search (case-insensitive)
        results = services.search_tasks("buy")
        assert len(results) == 2, "Should find 2 tasks with 'buy'"

        results = services.search_tasks("BUY")
        assert len(results) == 2, "Search should be case-insensitive"

        # Test filter by status
        completed = services.filter_by_status(completed=True)
        assert len(completed) == 1, "Should have 1 completed task"

        pending = services.filter_by_status(completed=False)
        assert len(pending) == 3, "Should have 3 pending tasks"

        # Test filter by priority
        high_priority = services.filter_by_priority("High")
        assert len(high_priority) == 2, "Should have 2 high priority tasks"

        # Test filter by tag
        shopping = services.filter_by_tag("shopping")
        assert len(shopping) == 2, "Should have 2 shopping tasks"

        work = services.filter_by_tag("work")
        assert len(work) == 2, "Should have 2 work tasks"

        print("  ✓ Search & Filter: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ Search & Filter: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ Search & Filter: ERROR - {e}")
        return False


def test_feature_8_sort_tasks():
    """Feature 8: Sort Tasks - Reorder by priority, date, title"""
    print("\n[8] Testing: Sort Tasks")

    try:
        setup()

        # Create tasks in random order
        services.create_task("Zebra task", priority="Low")
        services.create_task("Apple task", priority="High")
        services.create_task("Middle task", priority="Medium")

        tasks = services.read_all_tasks()

        # Test sort by priority
        sorted_priority = services.sort_tasks(tasks, by="priority", reverse=True)
        assert sorted_priority[0].priority == "High", "First should be High priority"
        assert sorted_priority[1].priority == "Medium", "Second should be Medium priority"
        assert sorted_priority[2].priority == "Low", "Third should be Low priority"

        # Test sort by title (A-Z)
        sorted_title = services.sort_tasks(tasks, by="title", reverse=False)
        assert sorted_title[0].title == "Apple task", "First should be 'Apple task'"
        assert sorted_title[2].title == "Zebra task", "Last should be 'Zebra task'"

        # Test sort by date
        sorted_date = services.sort_tasks(tasks, by="date", reverse=True)
        assert len(sorted_date) == 3, "Should have all 3 tasks"

        print("  ✓ Sort Tasks: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ Sort Tasks: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ Sort Tasks: ERROR - {e}")
        return False


def test_feature_9_recurring_tasks():
    """Feature 9: Recurring Tasks - Auto-reschedule repeating tasks"""
    print("\n[9] Testing: Recurring Tasks")

    try:
        setup()

        # Create recurring task
        task = services.create_task("Daily standup")
        task_id = task.id

        # Set daily recurrence
        start_date = datetime(2025, 12, 29, 10, 0)
        result = services.set_recurrence(task_id, "Daily", start_date)
        assert result == True, "Set recurrence should succeed"
        assert task.recurrence is not None, "Task should have recurrence"
        assert task.recurrence.frequency == "Daily", "Frequency should be Daily"

        # Test weekly recurrence calculation
        task2 = services.create_task("Weekly meeting")
        services.set_recurrence(task2.id, "Weekly", start_date)

        # Test monthly recurrence
        task3 = services.create_task("Monthly review")
        services.set_recurrence(task3.id, "Monthly", start_date)

        # Test recurring task regeneration on completion
        initial_count = len(services.read_all_tasks())
        services.toggle_completion(task_id)  # Mark complete

        # Check that new task was created
        new_count = len(services.read_all_tasks())
        assert new_count == initial_count + 1, "New task should be created on completion"

        # Verify new task has correct attributes
        all_tasks = services.read_all_tasks()
        new_recurring = [t for t in all_tasks if t.title == "Daily standup" and not t.completed]
        assert len(new_recurring) == 1, "Should have 1 new incomplete recurring task"
        assert new_recurring[0].recurrence is not None, "New task should have recurrence"

        print("  ✓ Recurring Tasks: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ Recurring Tasks: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ Recurring Tasks: ERROR - {e}")
        return False


def test_feature_10_due_dates_and_reminders():
    """Feature 10: Due Dates & Time Reminders - Set deadlines with notifications"""
    print("\n[10] Testing: Due Dates & Reminders")

    try:
        setup()

        # Create task and set due date
        task = services.create_task("Report deadline")
        task_id = task.id

        due_date = datetime(2025, 12, 30, 14, 0)
        result = services.set_due_date(task_id, due_date)
        assert result == True, "Set due date should succeed"
        assert task.due_date == due_date, "Due date should be set"

        # Test validate_date function
        parsed = services.validate_date("2025-12-30 14:00")
        assert parsed is not None, "Should parse ISO 8601 format"

        parsed = services.validate_date("2025-12-30")
        assert parsed is not None, "Should parse date-only format"

        parsed = services.validate_date("invalid")
        assert parsed is None, "Should return None for invalid format"

        # Test reminder detection (task due now)
        now = datetime.now()
        task_due_now = services.create_task("Due now task")
        services.set_due_date(task_due_now.id, now)

        reminders = services.check_reminders()
        assert len(reminders) >= 1, "Should detect task due now"

        # Test overdue detection
        past_date = now - timedelta(hours=1)
        task_overdue = services.create_task("Overdue task")
        services.set_due_date(task_overdue.id, past_date)

        is_late = services.is_overdue(task_overdue)
        assert is_late == True, "Should detect overdue task"

        # Test reminder doesn't show completed tasks
        services.toggle_completion(task_due_now.id)
        reminders = services.check_reminders()
        completed_in_reminders = any(t.id == task_due_now.id for t in reminders)
        assert completed_in_reminders == False, "Completed tasks should not show in reminders"

        print("  ✓ Due Dates & Reminders: PASS")
        return True

    except AssertionError as e:
        print(f"  ✗ Due Dates & Reminders: FAIL - {e}")
        return False
    except Exception as e:
        print(f"  ✗ Due Dates & Reminders: ERROR - {e}")
        return False


def run_validation():
    """Run all feature validation tests."""
    print("="*60)
    print("TODO APPLICATION - FEATURE VALIDATION")
    print("="*60)
    print("\nValidating all 10 required features:")
    print("  5 Basic + 3 Intermediate + 2 Advanced\n")

    results = []

    # Basic Level
    print("--- BASIC LEVEL (5 features) ---")
    results.append(("Add Task", test_feature_1_add_task()))
    results.append(("Delete Task", test_feature_2_delete_task()))
    results.append(("Update Task", test_feature_3_update_task()))
    results.append(("View Task List", test_feature_4_view_task_list()))
    results.append(("Mark as Complete", test_feature_5_mark_complete()))

    # Intermediate Level
    print("\n--- INTERMEDIATE LEVEL (3 features) ---")
    results.append(("Priorities & Tags", test_feature_6_priorities_and_tags()))
    results.append(("Search & Filter", test_feature_7_search_and_filter()))
    results.append(("Sort Tasks", test_feature_8_sort_tasks()))

    # Advanced Level
    print("\n--- ADVANCED LEVEL (2 features) ---")
    results.append(("Recurring Tasks", test_feature_9_recurring_tasks()))
    results.append(("Due Dates & Reminders", test_feature_10_due_dates_and_reminders()))

    # Summary
    print("\n" + "="*60)
    print("VALIDATION SUMMARY")
    print("="*60)

    passed = sum(1 for _, result in results if result)
    failed = sum(1 for _, result in results if not result)

    for name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {name:25} {status}")

    print("\n" + "="*60)
    print(f"Total Features: {len(results)}")
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print("="*60)

    if failed == 0:
        print("\n🎉 SUCCESS: All 10 features validated!")
        print("✅ READY FOR SUBMISSION")
        return 0
    else:
        print(f"\n⚠️  {failed} feature(s) failed validation")
        print("❌ NEEDS FIXES")
        return 1


if __name__ == "__main__":
    exit_code = run_validation()
    sys.exit(exit_code)
