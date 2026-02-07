"""
Test specific features: Sort Tasks, Set Due Date, Set Recurring Task
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


def test_sort_tasks():
    """Test Feature 8: Sort Tasks"""
    print("\n" + "="*60)
    print("[8] TESTING: Sort Tasks")
    print("="*60)

    storage.clear_storage()

    # Create tasks with different attributes
    print("\n1. Creating test tasks...")
    task1 = services.create_task("Zebra task", priority="Low", tags=["animals"])
    task2 = services.create_task("Apple task", priority="High", tags=["fruits"])
    task3 = services.create_task("Middle task", priority="Medium", tags=["general"])
    task4 = services.create_task("Banana task", priority="High", tags=["fruits"])

    print(f"   Created 4 tasks (IDs: {task1.id}, {task2.id}, {task3.id}, {task4.id})")

    # Test 1: Sort by Priority (High to Low)
    print("\n2. Sort by Priority (High → Medium → Low):")
    tasks = services.read_all_tasks()
    sorted_priority = services.sort_tasks(tasks, by="priority", reverse=True)

    for i, task in enumerate(sorted_priority, 1):
        print(f"   {i}. [{task.priority:6}] {task.title}")

    assert sorted_priority[0].priority == "High", "First should be High"
    assert sorted_priority[1].priority == "High", "Second should be High"
    assert sorted_priority[2].priority == "Medium", "Third should be Medium"
    assert sorted_priority[3].priority == "Low", "Fourth should be Low"
    print("   ✓ Priority sort: PASS")

    # Test 2: Sort by Title (A-Z)
    print("\n3. Sort by Title (A-Z):")
    sorted_title = services.sort_tasks(tasks, by="title", reverse=False)

    for i, task in enumerate(sorted_title, 1):
        print(f"   {i}. {task.title}")

    assert sorted_title[0].title == "Apple task", "First should be 'Apple task'"
    assert sorted_title[1].title == "Banana task", "Second should be 'Banana task'"
    assert sorted_title[2].title == "Middle task", "Third should be 'Middle task'"
    assert sorted_title[3].title == "Zebra task", "Fourth should be 'Zebra task'"
    print("   ✓ Title sort (A-Z): PASS")

    # Test 3: Sort by Title (Z-A)
    print("\n4. Sort by Title (Z-A):")
    sorted_title_rev = services.sort_tasks(tasks, by="title", reverse=True)

    for i, task in enumerate(sorted_title_rev, 1):
        print(f"   {i}. {task.title}")

    assert sorted_title_rev[0].title == "Zebra task", "First should be 'Zebra task'"
    assert sorted_title_rev[3].title == "Apple task", "Last should be 'Apple task'"
    print("   ✓ Title sort (Z-A): PASS")

    # Test 4: Sort by Date
    print("\n5. Sort by Date (Newest First):")
    sorted_date = services.sort_tasks(tasks, by="date", reverse=True)

    for i, task in enumerate(sorted_date, 1):
        print(f"   {i}. ID#{task.id} - {task.title} (Created: {task.created_at.strftime('%H:%M:%S')})")

    assert sorted_date[0].id == task4.id, "Newest task should be first"
    print("   ✓ Date sort: PASS")

    print("\n✅ Feature 8 (Sort Tasks): ALL TESTS PASSED\n")


def test_set_due_date():
    """Test Feature 9: Set Due Date"""
    print("\n" + "="*60)
    print("[9] TESTING: Set Due Date")
    print("="*60)

    storage.clear_storage()

    # Create test task
    print("\n1. Creating test task...")
    task = services.create_task("Report deadline", priority="High", tags=["work"])
    print(f"   Task created: {task}")

    # Test 1: Set due date
    print("\n2. Setting due date (2025-12-30 14:00)...")
    due_date = datetime(2025, 12, 30, 14, 0)
    result = services.set_due_date(task.id, due_date)

    assert result == True, "Set due date should succeed"
    assert task.due_date == due_date, "Due date should be set correctly"
    print(f"   ✓ Due date set successfully")
    print(f"   Task now: {task}")

    # Test 2: Parse date string (ISO 8601)
    print("\n3. Testing date parsing...")
    parsed1 = services.validate_date("2025-12-31 23:59")
    assert parsed1 is not None, "Should parse ISO 8601 with time"
    assert parsed1.year == 2025 and parsed1.month == 12 and parsed1.day == 31
    print(f"   ✓ Parsed: '2025-12-31 23:59' → {parsed1}")

    parsed2 = services.validate_date("2026-01-15")
    assert parsed2 is not None, "Should parse date-only format"
    print(f"   ✓ Parsed: '2026-01-15' → {parsed2}")

    parsed3 = services.validate_date("invalid-date")
    assert parsed3 is None, "Should return None for invalid format"
    print(f"   ✓ Invalid format rejected: 'invalid-date' → None")

    # Test 3: Overdue detection
    print("\n4. Testing overdue detection...")
    past_task = services.create_task("Overdue task")
    past_date = datetime.now() - timedelta(hours=2)
    services.set_due_date(past_task.id, past_date)

    is_late = services.is_overdue(past_task)
    assert is_late == True, "Should detect overdue task"
    print(f"   ✓ Overdue detected: {past_task.title} (due {past_date.strftime('%Y-%m-%d %H:%M')})")
    print(f"   Task display: {past_task}")

    # Test 4: Reminder detection
    print("\n5. Testing reminder system...")
    reminder_task = services.create_task("Due now task")
    now = datetime.now()
    services.set_due_date(reminder_task.id, now)

    reminders = services.check_reminders()
    assert len(reminders) >= 1, "Should detect task due now"
    print(f"   ✓ Reminders detected: {len(reminders)} task(s) due within 5 minutes")

    for rem_task in reminders:
        print(f"     - {rem_task.title}")

    # Test 5: Completed tasks not in reminders
    services.toggle_completion(reminder_task.id)
    reminders_after = services.check_reminders()
    assert reminder_task.id not in [t.id for t in reminders_after], "Completed task should not show in reminders"
    print(f"   ✓ Completed tasks excluded from reminders")

    print("\n✅ Feature 9 (Set Due Date): ALL TESTS PASSED\n")


def test_set_recurring_task():
    """Test Feature 10: Set Recurring Task"""
    print("\n" + "="*60)
    print("[10] TESTING: Set Recurring Task")
    print("="*60)

    storage.clear_storage()

    # Test 1: Daily recurrence
    print("\n1. Testing Daily recurrence...")
    daily_task = services.create_task("Daily standup", priority="Medium", tags=["work"])
    start_date = datetime(2025, 12, 29, 10, 0)

    result = services.set_recurrence(daily_task.id, "Daily", start_date)
    assert result == True, "Set recurrence should succeed"
    assert daily_task.recurrence is not None, "Task should have recurrence"
    assert daily_task.recurrence.frequency == "Daily", "Frequency should be Daily"
    assert daily_task.due_date == start_date, "Due date should be set to start date"

    print(f"   ✓ Daily recurrence set")
    print(f"   Task: {daily_task}")

    # Test 2: Weekly recurrence
    print("\n2. Testing Weekly recurrence...")
    weekly_task = services.create_task("Weekly team meeting", priority="High", tags=["work", "meeting"])
    services.set_recurrence(weekly_task.id, "Weekly", start_date)

    assert weekly_task.recurrence.frequency == "Weekly", "Frequency should be Weekly"
    print(f"   ✓ Weekly recurrence set")
    print(f"   Task: {weekly_task}")

    # Test 3: Monthly recurrence
    print("\n3. Testing Monthly recurrence...")
    monthly_task = services.create_task("Monthly review", priority="High", tags=["admin"])
    services.set_recurrence(monthly_task.id, "Monthly", start_date)

    assert monthly_task.recurrence.frequency == "Monthly", "Frequency should be Monthly"
    print(f"   ✓ Monthly recurrence set")
    print(f"   Task: {monthly_task}")

    # Test 4: Task regeneration on completion
    print("\n4. Testing auto-regeneration on completion...")
    initial_count = len(services.read_all_tasks())
    print(f"   Initial task count: {initial_count}")

    # Mark daily task complete
    services.toggle_completion(daily_task.id)
    print(f"   Marked task #{daily_task.id} as complete")

    new_count = len(services.read_all_tasks())
    print(f"   New task count: {new_count}")

    assert new_count == initial_count + 1, f"Expected {initial_count + 1} tasks, got {new_count}"
    print(f"   ✓ New task created automatically")

    # Find the new task
    all_tasks = services.read_all_tasks()
    new_recurring = [t for t in all_tasks if t.title == "Daily standup" and not t.completed]

    assert len(new_recurring) == 1, "Should have exactly 1 new incomplete recurring task"
    new_task = new_recurring[0]

    print(f"\n   New task details:")
    print(f"     ID: #{new_task.id}")
    print(f"     Title: {new_task.title}")
    print(f"     Completed: {new_task.completed}")
    print(f"     Due Date: {new_task.due_date.strftime('%Y-%m-%d %H:%M')}")
    print(f"     Recurrence: {new_task.recurrence.frequency}")

    # Verify next due date is calculated correctly
    expected_next = start_date + timedelta(days=1)
    assert new_task.due_date == expected_next, "Next due date should be 1 day later"
    print(f"   ✓ Next due date calculated correctly: {new_task.due_date.strftime('%Y-%m-%d %H:%M')}")

    # Test 5: Next occurrence calculation
    print("\n5. Testing date calculations for all frequencies...")

    # Daily
    next_daily = services.calculate_next_occurrence(start_date, "Daily")
    assert next_daily == start_date + timedelta(days=1), "Daily should add 1 day"
    print(f"   ✓ Daily: {start_date.strftime('%Y-%m-%d')} → {next_daily.strftime('%Y-%m-%d')}")

    # Weekly
    next_weekly = services.calculate_next_occurrence(start_date, "Weekly")
    assert next_weekly == start_date + timedelta(weeks=1), "Weekly should add 7 days"
    print(f"   ✓ Weekly: {start_date.strftime('%Y-%m-%d')} → {next_weekly.strftime('%Y-%m-%d')}")

    # Monthly
    next_monthly = services.calculate_next_occurrence(start_date, "Monthly")
    assert next_monthly.month == 1 and next_monthly.year == 2026, "Monthly should go to next month"
    print(f"   ✓ Monthly: {start_date.strftime('%Y-%m-%d')} → {next_monthly.strftime('%Y-%m-%d')}")

    print("\n✅ Feature 10 (Set Recurring Task): ALL TESTS PASSED\n")


if __name__ == "__main__":
    print("="*60)
    print("TESTING FEATURES 8, 9, 10")
    print("="*60)

    try:
        test_sort_tasks()
        test_set_due_date()
        test_set_recurring_task()

        print("="*60)
        print("🎉 ALL 3 FEATURES VALIDATED SUCCESSFULLY!")
        print("="*60)
        print("\n✅ Feature 8 (Sort Tasks): PASS")
        print("✅ Feature 9 (Set Due Date): PASS")
        print("✅ Feature 10 (Set Recurring Task): PASS")
        print("\n" + "="*60)

    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
