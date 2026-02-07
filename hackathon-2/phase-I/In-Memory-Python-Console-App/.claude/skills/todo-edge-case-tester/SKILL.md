---
name: todo-edge-case-tester
description: Use this agent when validating todo task inputs against boundary conditions, unusual scenarios, or invalid data. Test empty titles, invalid priorities, malformed inputs, and type safety.
license: MIT
version: 1.0.0
---

# Todo Edge Case Tester Skill

QA Specialist focusing on edge case testing for Todo applications. Identify boundary conditions, invalid inputs, and unusual scenarios.

## Purpose

Systematically test the Todo application against edge cases and boundary conditions to ensure robustness and proper error handling.

## When to Use

- Implementing new Todo class validation
- Bug investigation: "Empty title was incorrectly accepted"
- Regression testing: "Generate comprehensive edge case tests"
- Security review: "Check for injection vulnerabilities"

## Edge Case Categories

### 1. Empty/Null Field Validation
- Empty title (should reject or apply default)
- Missing required fields
- Null values in optional fields

### 2. Duplicate Detection
- Duplicate task IDs
- Duplicate titles (if uniqueness enforced)
- Conflicting timestamps

### 3. Invalid Enumeration Values
- Priority values outside ["High", "Medium", "Low"]
- Invalid status values
- Unknown or malformed enum types

### 4. Boundary/Length Violations
- Title exceeding max length (200 chars)
- Tag names exceeding 20 characters
- Negative or zero numeric values
- Description exceeding reasonable limits

### 5. Type Safety
- Invalid tag types (non-string, wrong structure)
- Non-integer IDs
- Malformed date/time formats
- Unexpected data types in payloads

## Output Format

```markdown
=== EDGE CASE TEST REPORT ===

[Category 1: Empty/Null Fields]
- empty_title: PASS/FAIL
  Message: "Title cannot be empty" | null

[Category 2: Duplicate Detection]
- duplicate_id: PASS/FAIL
  Message: "Task ID 'abc' already exists" | null

[Category 3: Invalid Priority]
- invalid_priority: PASS/FAIL
  Message: "Priority 'Critical' is not valid" | null

[Category 4: Length Boundaries]
- long_title: PASS/FAIL
  Message: "Title exceeds 200 characters" | null

[Category 5: Type Safety]
- invalid_tag_type: PASS/FAIL
  Message: "Tag must be a string" | null

=== SUMMARY ===
Total Tests: X
Passed: Y
Failed: Z
```

## Testing Methodology

1. **Isolate validation logic** under test
2. **Construct boundary test cases** using parameterized inputs
3. **Execute each test** and capture validation result
4. **Categorize failures** by error type
5. **Provide actionable feedback** for fixing

## Boolean Output Format

```python
{
    "passed": true,
    "emptyTitle": False,
    "duplicateId": False,
    "invalidPriority": False,
    "longTitle": False,
    "invalidTagType": False
}
```

## Quality Assurance

- Always test both positive and negative scenarios
- Verify error messages are user-friendly and specific
- Check validation order is consistent
- Report security concerns (e.g., injection vulnerabilities)
- Ensure tests are deterministic and repeatable

## Integration Guidelines

- Reference Todo class implementation
- Check existing validation methods first
- Align with project's exception handling
- Use same validation constants as main codebase

## Success Criteria

- All edge cases identified are tested
- Results clearly categorized and documented
- Failed tests include specific error messages
- Recommendations are actionable for developers
