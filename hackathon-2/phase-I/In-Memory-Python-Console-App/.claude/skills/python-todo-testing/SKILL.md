---
name: python-todo-testing
description: Guide for testing in-memory Python Todo applications. This skill should be used when validating logic and handling edge cases.
license: MIT
---

# Python Todo Testing Skill

Comprehensive testing strategy for in-memory Todo applications.

## Purpose
Ensure correctness of core functionality, validate edge cases, and maintain code quality through systematic testing.

## When to Use
- Testing core CRUD operations
- Validating search and filter logic
- Testing edge cases and error handling
- Verifying data integrity
- Regression testing after changes

## Core Responsibilities
1. **Unit Testing** - Test individual functions
2. **Integration Testing** - Test component interactions
3. **Edge Case Testing** - Handle boundary conditions
4. **Error Testing** - Verify error handling
5. **Data Validation** - Ensure data integrity

## Testing Strategy

### 1. Test Structure
```python
# test_core.py
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import (
    add_task, get_all_tasks, get_task_by_id,
    update_task, delete_task, toggle_task_completion,
    clear_all_tasks, get_task_count
)

def setup_function():
    """Run before each test - clear all tasks"""
    clear_all_tasks()

def teardown_function():
    """Run after each test - cleanup"""
    clear_all_tasks()
```

### 2. Core CRUD Tests
```python
# Test: Add Task
def test_add_task_success():
    """Test adding a task successfully"""
    task = add_task("Buy groceries", priority="High", tags=["shopping"])

    assert task is not None
    assert task.id > 0
    assert task.title == "Buy groceries"
    assert task.priority == "High"
    assert task.completed == False
    assert "shopping" in task.tags

    print("✓ test_add_task_success passed")

def test_add_task_with_defaults():
    """Test adding task with default values"""
    task = add_task("Simple task")

    assert task.priority == "Medium"
    assert task.tags == []
    assert task.completed == False

    print("✓ test_add_task_with_defaults passed")

def test_add_task_empty_title():
    """Test that empty title raises ValueError"""
    try:
        add_task("")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "empty" in str(e).lower()
        print("✓ test_add_task_empty_title passed")

def test_add_task_invalid_priority():
    """Test that invalid priority raises ValueError"""
    try:
        add_task("Task", priority="Critical")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "priority" in str(e).lower()
        print("✓ test_add_task_invalid_priority passed")

# Test: Get Tasks
def test_get_all_tasks_empty():
    """Test getting tasks when none exist"""
    tasks = get_all_tasks()
    assert tasks == []
    print("✓ test_get_all_tasks_empty passed")

def test_get_all_tasks_multiple():
    """Test getting multiple tasks"""
    add_task("Task 1")
    add_task("Task 2")
    add_task("Task 3")

    tasks = get_all_tasks()
    assert len(tasks) == 3
    print("✓ test_get_all_tasks_multiple passed")

def test_get_task_by_id_exists():
    """Test retrieving existing task by ID"""
    task = add_task("Find me")
    retrieved = get_task_by_id(task.id)

    assert retrieved is not None
    assert retrieved.id == task.id
    assert retrieved.title == "Find me"
    print("✓ test_get_task_by_id_exists passed")

def test_get_task_by_id_not_exists():
    """Test retrieving non-existent task"""
    retrieved = get_task_by_id(999)
    assert retrieved is None
    print("✓ test_get_task_by_id_not_exists passed")

# Test: Update Task
def test_update_task_title():
    """Test updating task title"""
    task = add_task("Original title")
    success = update_task(task.id, title="Updated title")

    assert success == True
    updated = get_task_by_id(task.id)
    assert updated.title == "Updated title"
    print("✓ test_update_task_title passed")

def test_update_task_priority():
    """Test updating task priority"""
    task = add_task("Task", priority="Low")
    success = update_task(task.id, priority="High")

    assert success == True
    updated = get_task_by_id(task.id)
    assert updated.priority == "High"
    print("✓ test_update_task_priority passed")

def test_update_task_not_found():
    """Test updating non-existent task"""
    success = update_task(999, title="New title")
    assert success == False
    print("✓ test_update_task_not_found passed")

def test_update_task_empty_title():
    """Test that empty title update raises error"""
    task = add_task("Original")

    try:
        update_task(task.id, title="")
        assert False, "Should have raised ValueError"
    except ValueError:
        print("✓ test_update_task_empty_title passed")

# Test: Delete Task
def test_delete_task_exists():
    """Test deleting existing task"""
    task = add_task("Delete me")
    success = delete_task(task.id)

    assert success == True
    assert get_task_by_id(task.id) is None
    print("✓ test_delete_task_exists passed")

def test_delete_task_not_exists():
    """Test deleting non-existent task"""
    success = delete_task(999)
    assert success == False
    print("✓ test_delete_task_not_exists passed")

def test_delete_task_preserves_others():
    """Test that deleting one task doesn't affect others"""
    task1 = add_task("Task 1")
    task2 = add_task("Task 2")
    task3 = add_task("Task 3")

    delete_task(task2.id)

    assert get_task_by_id(task1.id) is not None
    assert get_task_by_id(task2.id) is None
    assert get_task_by_id(task3.id) is not None
    print("✓ test_delete_task_preserves_others passed")

# Test: Toggle Completion
def test_toggle_completion_to_complete():
    """Test marking task as completed"""
    task = add_task("Complete me")
    success = toggle_task_completion(task.id)

    assert success == True
    updated = get_task_by_id(task.id)
    assert updated.completed == True
    print("✓ test_toggle_completion_to_complete passed")

def test_toggle_completion_to_incomplete():
    """Test unmarking completed task"""
    task = add_task("Toggle me")
    toggle_task_completion(task.id)  # Complete
    toggle_task_completion(task.id)  # Uncomplete

    updated = get_task_by_id(task.id)
    assert updated.completed == False
    print("✓ test_toggle_completion_to_incomplete passed")

def test_toggle_completion_not_found():
    """Test toggling non-existent task"""
    success = toggle_task_completion(999)
    assert success == False
    print("✓ test_toggle_completion_not_found passed")

# Test: Statistics
def test_get_task_count_empty():
    """Test statistics with no tasks"""
    stats = get_task_count()

    assert stats['total'] == 0
    assert stats['completed'] == 0
    assert stats['pending'] == 0
    print("✓ test_get_task_count_empty passed")

def test_get_task_count_mixed():
    """Test statistics with mixed tasks"""
    task1 = add_task("Task 1")
    task2 = add_task("Task 2")
    task3 = add_task("Task 3")

    toggle_task_completion(task1.id)
    toggle_task_completion(task2.id)

    stats = get_task_count()
    assert stats['total'] == 3
    assert stats['completed'] == 2
    assert stats['pending'] == 1
    print("✓ test_get_task_count_mixed passed")
```

### 3. Organization Feature Tests
```python
# test_organization.py
from organization import (
    search_tasks_by_keyword,
    filter_by_status,
    filter_by_priority,
    sort_tasks_by_priority,
    get_all_tags,
    add_tag_to_task
)

def test_search_by_keyword():
    """Test keyword search"""
    add_task("Buy groceries")
    add_task("Buy books")
    add_task("Read documentation")

    results = search_tasks_by_keyword("buy")
    assert len(results) == 2
    print("✓ test_search_by_keyword passed")

def test_search_case_insensitive():
    """Test that search is case-insensitive"""
    add_task("Important Task")

    results = search_tasks_by_keyword("IMPORTANT")
    assert len(results) == 1
    print("✓ test_search_case_insensitive passed")

def test_filter_by_status_completed():
    """Test filtering completed tasks"""
    task1 = add_task("Task 1")
    task2 = add_task("Task 2")

    toggle_task_completion(task1.id)

    completed = filter_by_status(completed=True)
    assert len(completed) == 1
    assert completed[0].id == task1.id
    print("✓ test_filter_by_status_completed passed")

def test_filter_by_priority():
    """Test filtering by priority"""
    add_task("Low priority", priority="Low")
    add_task("High priority", priority="High")
    add_task("Another high", priority="High")

    high_tasks = filter_by_priority("High")
    assert len(high_tasks) == 2
    print("✓ test_filter_by_priority passed")

def test_sort_by_priority():
    """Test sorting by priority"""
    add_task("Low task", priority="Low")
    add_task("High task", priority="High")
    add_task("Medium task", priority="Medium")

    sorted_tasks = sort_tasks_by_priority()

    assert sorted_tasks[0].priority == "High"
    assert sorted_tasks[1].priority == "Medium"
    assert sorted_tasks[2].priority == "Low"
    print("✓ test_sort_by_priority passed")

def test_add_tag():
    """Test adding tag to task"""
    task = add_task("Task with tags")
    success = add_tag_to_task(task.id, "urgent")

    assert success == True
    updated = get_task_by_id(task.id)
    assert "urgent" in updated.tags
    print("✓ test_add_tag passed")

def test_get_all_tags():
    """Test retrieving all unique tags"""
    add_task("Task 1", tags=["work", "urgent"])
    add_task("Task 2", tags=["personal", "urgent"])

    all_tags = get_all_tags()
    assert len(all_tags) == 3
    assert "work" in all_tags
    assert "personal" in all_tags
    assert "urgent" in all_tags
    print("✓ test_get_all_tags passed")
```

### 4. Edge Case Tests
```python
# test_edge_cases.py

def test_add_many_tasks():
    """Test adding large number of tasks"""
    for i in range(100):
        add_task(f"Task {i}")

    tasks = get_all_tasks()
    assert len(tasks) == 100
    print("✓ test_add_many_tasks passed")

def test_task_with_special_characters():
    """Test task with special characters in title"""
    task = add_task("Task with @#$% & symbols!")
    assert task.title == "Task with @#$% & symbols!"
    print("✓ test_task_with_special_characters passed")

def test_task_with_very_long_title():
    """Test task with long title"""
    long_title = "A" * 1000
    task = add_task(long_title)
    assert len(task.title) == 1000
    print("✓ test_task_with_very_long_title passed")

def test_task_with_whitespace_title():
    """Test that whitespace is trimmed"""
    task = add_task("  Task with spaces  ")
    assert task.title == "Task with spaces"
    print("✓ test_task_with_whitespace_title passed")

def test_multiple_operations_on_same_task():
    """Test multiple operations on one task"""
    task = add_task("Test task", priority="Low")

    # Update
    update_task(task.id, priority="High")

    # Toggle
    toggle_task_completion(task.id)

    # Add tag
    add_tag_to_task(task.id, "test")

    # Verify
    final = get_task_by_id(task.id)
    assert final.priority == "High"
    assert final.completed == True
    assert "test" in final.tags

    print("✓ test_multiple_operations_on_same_task passed")

def test_delete_all_tasks():
    """Test clearing all tasks"""
    add_task("Task 1")
    add_task("Task 2")
    add_task("Task 3")

    clear_all_tasks()

    tasks = get_all_tasks()
    assert len(tasks) == 0
    print("✓ test_delete_all_tasks passed")

def test_id_uniqueness():
    """Test that task IDs are unique"""
    task1 = add_task("Task 1")
    task2 = add_task("Task 2")
    task3 = add_task("Task 3")

    ids = {task1.id, task2.id, task3.id}
    assert len(ids) == 3  # All unique
    print("✓ test_id_uniqueness passed")
```

### 5. Test Runner
```python
# run_tests.py
"""
Simple test runner for Todo application
"""

def run_all_tests():
    """Run all test modules"""
    print("="*60)
    print("RUNNING TODO APPLICATION TESTS")
    print("="*60)

    # Track results
    passed = 0
    failed = 0
    errors = []

    # Import test modules
    try:
        import test_core
        import test_organization
        import test_edge_cases
    except ImportError as e:
        print(f"✗ Failed to import test modules: {e}")
        return

    # Get all test functions
    test_modules = [test_core, test_organization, test_edge_cases]
    all_tests = []

    for module in test_modules:
        tests = [
            (module.__name__, name, getattr(module, name))
            for name in dir(module)
            if name.startswith('test_') and callable(getattr(module, name))
        ]
        all_tests.extend(tests)

    print(f"\nFound {len(all_tests)} tests\n")

    # Run each test
    for module_name, test_name, test_func in all_tests:
        try:
            # Setup
            if hasattr(test_core, 'setup_function'):
                test_core.setup_function()

            # Run test
            test_func()
            passed += 1

            # Teardown
            if hasattr(test_core, 'teardown_function'):
                test_core.teardown_function()

        except AssertionError as e:
            failed += 1
            errors.append((module_name, test_name, str(e)))
            print(f"✗ {test_name} FAILED: {e}")

        except Exception as e:
            failed += 1
            errors.append((module_name, test_name, str(e)))
            print(f"✗ {test_name} ERROR: {e}")

    # Print summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    print(f"Total Tests: {len(all_tests)}")
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")

    if failed > 0:
        print(f"\nFailures:")
        for module, test, error in errors:
            print(f"  - {module}.{test}: {error}")

    print("="*60)

    return failed == 0

if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)
```

### 6. Manual Testing Checklist
```markdown
## Manual Test Cases

### Basic Operations
- [ ] Add task with title only
- [ ] Add task with priority and tags
- [ ] View empty task list
- [ ] View task list with multiple tasks
- [ ] Update task title
- [ ] Update task priority
- [ ] Delete task
- [ ] Toggle task completion

### Edge Cases
- [ ] Add task with empty title (should fail)
- [ ] Add task with invalid priority (should fail)
- [ ] Update non-existent task (should fail gracefully)
- [ ] Delete non-existent task (should fail gracefully)
- [ ] Add 50+ tasks (performance check)

### Search & Filter
- [ ] Search with keyword
- [ ] Search with no results
- [ ] Filter by completed status
- [ ] Filter by priority
- [ ] Sort by priority
- [ ] Sort by date

### UI/UX
- [ ] Menu displays correctly
- [ ] Error messages are clear
- [ ] Success confirmations shown
- [ ] Task details formatted properly
- [ ] Can exit application cleanly
```

## Testing Best Practices
- **Isolation** - Each test is independent
- **Setup/Teardown** - Clean state before/after tests
- **Clear Names** - Test names describe what they test
- **One Assertion** - Focus on one thing per test
- **Edge Cases** - Test boundaries and error conditions

## Common Test Patterns
```python
# Arrange - Set up test data
task = add_task("Test task")

# Act - Perform operation
success = delete_task(task.id)

# Assert - Verify result
assert success == True
assert get_task_by_id(task.id) is None
```

## Anti-Patterns to Avoid
- Tests that depend on execution order
- Not cleaning up after tests
- Testing multiple things in one test
- Ignoring edge cases
- No error case testing

## Integration with Development
- Run tests after every change
- Write tests before fixing bugs
- Add tests for new features
- Use tests to document expected behavior

## Next Steps
1. Implement automated test suite
2. Add code coverage reporting
3. Set up continuous testing
4. Create performance benchmarks
