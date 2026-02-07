# Implementation Plan: Todo In-Memory Python Console App

**Branch**: `001-todo-app` | **Date**: 2025-12-29 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-todo-app/spec.md`

**Note**: This plan follows the spec-driven development workflow using Claude Code and Spec-Kit Plus.

## Summary

Build a fully functional in-memory Python console Todo application with three feature tiers: Basic (CRUD operations), Intermediate (priorities, tags, search, filter, sort), and Advanced (due dates, reminders, recurring tasks). All task data stored in memory using Python lists/dictionaries with no persistence. CLI-only interface with numbered menus, input validation, and clear error messages. UV package manager handles all dependencies. Modular structure (models.py, storage.py, services.py, cli.py, main.py, tests/) enables independent testing and parallel development.

## Technical Context

**Language/Version**: Python 3.13+
**Primary Dependencies**:
- Standard library (datetime, dataclasses, typing)
- Optional: colorama (enhanced console output)
- Optional: pytest (testing framework)
- UV package manager (dependency management)

**Storage**: In-memory (Python lists and dictionaries - NO persistence)
**Testing**: pytest for unit and integration tests
**Target Platform**: Cross-platform (Windows, macOS, Linux) console application
**Project Type**: single (console CLI application)
**Performance Goals**:
- Task operations complete in < 2 seconds for 1000 tasks
- Search/filter return results in < 1 second
- Application startup < 1 second

**Constraints**:
- In-memory only (ephemeral data, resets on exit)
- CLI-only interface (no GUI, no web)
- Single-user, single-session
- No external services/APIs
- No file/database persistence

**Scale/Scope**:
- Support 100-1000 tasks with reasonable performance
- 10 required features (5 Basic + 3 Intermediate + 2 Advanced)
- 5 core modules + test suite
- ~2000-3000 lines of code estimated

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Spec-Driven Development ✅ PASS
- ✅ Feature specification complete (spec.md)
- ✅ Architecture plan being documented (this file)
- ✅ Tasks will be broken down (tasks.md via /sp.tasks)
- ✅ PHRs created for each phase
- ✅ ADRs will be created for significant decisions

### II. In-Memory Architecture ✅ PASS
- ✅ All data stored in Python lists/dictionaries
- ✅ NO file persistence
- ✅ NO database connections
- ✅ Ephemeral data (resets on exit)
- ✅ Storage layer separated from business logic

### III. Modular Python Structure ✅ PASS
- ✅ models.py - Task dataclass definitions
- ✅ storage.py - In-memory storage management
- ✅ services.py - Business logic and CRUD
- ✅ cli.py - User interface and menus
- ✅ main.py - Application entry point
- ✅ tests/ - Test modules by layer

### IV. Feature Completeness ✅ PASS
- ✅ All 5 Basic features planned
- ✅ All 3 Intermediate features planned
- ✅ All 2 Advanced features planned
- ✅ Phased implementation (P1 → P2 → P3)

### V. Testing & Quality ✅ PASS
- ✅ Unit tests for services.py planned
- ✅ Integration tests for CLI workflows planned
- ✅ Edge case coverage planned
- ✅ Test organization mirrors module structure

### VI. Console-First UX ✅ PASS
- ✅ CLI-only interface
- ✅ Numbered menu options
- ✅ Input validation planned
- ✅ Confirmation prompts for destructive actions
- ✅ Readable formatting (tables, indicators)

### VII. Reproducibility ✅ PASS
- ✅ Deterministic behavior (sequential IDs)
- ✅ Clear state management (in-memory)
- ✅ Predictable operations
- ✅ No hidden state or side effects

**Gate Status**: ✅ ALL GATES PASSED - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/001-todo-app/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file (in progress)
├── research.md          # Phase 0 output - Technical research
├── data-model.md        # Phase 1 output - Task entity design
├── quickstart.md        # Phase 1 output - User guide
├── contracts/           # Phase 1 output - Module APIs
│   ├── models-api.md
│   ├── storage-api.md
│   ├── services-api.md
│   └── cli-api.md
├── checklists/          # Quality validation
│   └── requirements.md  # Spec quality checklist (complete)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
src/
├── models.py           # Task dataclass with all attributes
├── storage.py          # In-memory list/dict management
├── services.py         # CRUD operations and business logic
├── cli.py              # Menu system and user interaction
└── main.py             # Application entry point

tests/
├── unit/
│   ├── test_models.py      # Task model validation
│   ├── test_storage.py     # Storage operations
│   └── test_services.py    # Business logic
├── integration/
│   └── test_cli.py         # End-to-end workflows
└── edge_cases/
    └── test_edge_cases.py  # Boundary conditions

pyproject.toml          # UV project configuration
README.md               # Setup and usage instructions
CLAUDE.md               # Claude Code workflow guide
```

**Structure Decision**: Single-project structure selected (Option 1) because:
- Console application without web/mobile components
- All code in one cohesive module hierarchy
- Simplified testing and deployment
- Matches constitution principle III (Modular Python Structure)

## Complexity Tracking

> No constitution violations - complexity tracking not required.

---

## Phase 0: Research & Technical Decisions

### Research Topics

1. **UV Package Manager Integration**
   - Decision: Use UV for all dependency management
   - Rationale: Modern, fast Python package manager; better dependency resolution than pip
   - Actions: Research UV project initialization, dependency specification, lock files

2. **In-Memory Storage Strategy**
   - Decision: Primary list + ID index dict for fast lookups
   - Rationale: List maintains insertion order; dict enables O(1) ID lookups
   - Actions: Research Python list/dict best practices, memory efficiency

3. **Recurring Task Implementation**
   - Decision: Calculate next occurrence on completion, store pattern with task
   - Rationale: Simpler than background scheduler; aligns with assumption #2 in spec
   - Actions: Research datetime arithmetic for Daily/Weekly/Monthly patterns

4. **Console Reminder System**
   - Decision: Check due dates on each menu display, show notifications inline
   - Rationale: No background threads needed; reminder displays when app is active
   - Actions: Research datetime comparison, console formatting for notifications

5. **ID Generation Strategy**
   - Decision: Global counter starting at 1, never reused
   - Rationale: Simple, predictable, matches assumption #4 in spec
   - Actions: Research thread-safe counter patterns (future-proofing)

6. **Date/Time Input Parsing**
   - Decision: Support ISO 8601 (YYYY-MM-DD HH:MM) and common variants
   - Rationale: Standard format, easy to parse, matches assumption #9
   - Actions: Research Python datetime.strptime patterns, validation

### Research Output Location

All research findings documented in: `specs/001-todo-app/research.md`

---

## Phase 1: Design & Contracts

### Data Model Design

**Primary Entity: Task**

Located in: `src/models.py`

```python
@dataclass
class Task:
    id: int                          # Unique, auto-generated, sequential
    title: str                       # Required, non-empty
    completed: bool = False          # Default: incomplete
    priority: str = "Medium"         # High/Medium/Low
    tags: list[str] = field(default_factory=list)
    created_at: datetime = field(default_factory=datetime.now)
    due_date: Optional[datetime] = None
    recurrence: Optional[RecurrencePattern] = None

@dataclass
class RecurrencePattern:
    frequency: str                   # None/Daily/Weekly/Monthly
    original_due_date: datetime      # Start date for pattern
```

**Secondary Entity: RecurrencePattern**

Embedded within Task, manages recurring task logic.

### Module API Contracts

#### 1. models.py API

**Purpose**: Define Task and RecurrencePattern dataclasses

**Public API**:
- `Task` dataclass with all attributes
- `RecurrencePattern` dataclass
- `to_dict()` method for serialization (future use)

**Dependencies**: None (standard library only)

#### 2. storage.py API

**Purpose**: Manage in-memory task storage

**Public API**:
```python
def initialize_storage() -> None
def add_task_to_storage(task: Task) -> None
def get_task_by_id(task_id: int) -> Optional[Task]
def get_all_tasks() -> list[Task]
def update_task_in_storage(task: Task) -> bool
def delete_task_from_storage(task_id: int) -> bool
def clear_storage() -> None
def get_next_id() -> int
```

**Data Structures**:
- `tasks_list: list[Task]` - Primary storage
- `tasks_index: dict[int, Task]` - Fast ID lookups
- `next_id_counter: int` - ID generation

#### 3. services.py API

**Purpose**: Business logic and CRUD operations

**Public API**:
```python
# Basic operations
def create_task(title: str, priority: str = "Medium",
                tags: list[str] = None) -> Task
def read_task(task_id: int) -> Optional[Task]
def read_all_tasks() -> list[Task]
def update_task(task_id: int, title: str = None,
                priority: str = None, tags: list[str] = None) -> bool
def delete_task(task_id: int) -> bool
def toggle_completion(task_id: int) -> bool

# Organization operations
def search_tasks(keyword: str) -> list[Task]
def filter_by_status(completed: bool) -> list[Task]
def filter_by_priority(priority: str) -> list[Task]
def filter_by_tag(tag: str) -> list[Task]
def sort_tasks(tasks: list[Task], by: str, reverse: bool = False) -> list[Task]

# Advanced operations
def set_due_date(task_id: int, due_date: datetime) -> bool
def set_recurrence(task_id: int, frequency: str, start_date: datetime) -> bool
def check_reminders() -> list[Task]  # Returns tasks due now
def regenerate_recurring_task(task_id: int) -> Optional[Task]

# Validation
def validate_title(title: str) -> bool
def validate_priority(priority: str) -> bool
def validate_date(date_str: str) -> Optional[datetime]
```

#### 4. cli.py API

**Purpose**: User interface and menu handling

**Public API**:
```python
def run_app() -> None                     # Main application loop
def display_main_menu() -> None
def handle_add_task() -> None
def handle_view_tasks() -> None
def handle_update_task() -> None
def handle_delete_task() -> None
def handle_toggle_completion() -> None
def handle_search() -> None
def handle_filter() -> None
def handle_sort() -> None
def handle_set_due_date() -> None
def handle_set_recurrence() -> None
def display_task_list(tasks: list[Task]) -> None
def display_task_details(task: Task) -> None
def get_user_input(prompt: str) -> str
def get_menu_choice() -> str
def confirm_action(message: str) -> bool
```

### Quickstart Guide Outline

**Location**: `specs/001-todo-app/quickstart.md`

**Sections**:
1. Installation (UV setup, dependencies)
2. Running the Application
3. Basic Operations Walkthrough
4. Organization Features Guide
5. Advanced Features Guide
6. Troubleshooting

---

## Phase 2: Implementation Phases

### Phase 2.1: Basic Level (Core Essentials) - User Story P1

**Objective**: Implement foundational CRUD operations

**Modules**:
1. `models.py` - Task dataclass (basic fields: id, title, completed, created_at)
2. `storage.py` - In-memory storage with list and dict
3. `services.py` - create, read, update, delete, toggle functions
4. `cli.py` - Main menu, basic handlers
5. `main.py` - Entry point

**Features**:
- Add Task (FR-001, FR-002)
- View Task List (FR-003)
- Update Task (FR-004)
- Mark as Complete (FR-005)
- Delete Task (FR-006, FR-007)

**Testing**:
- Unit tests for services.py CRUD operations
- Integration test for basic CLI workflow
- Edge cases: empty title, invalid ID, delete confirmation

**Deliverable**: Functional MVP with core task management

**Estimated Complexity**: Medium (foundation for all features)

### Phase 2.2: Intermediate Level (Organization) - User Story P2

**Objective**: Add priorities, tags, search, filter, sort

**Module Updates**:
1. `models.py` - Add priority and tags fields
2. `services.py` - Add organization functions (search, filter, sort)
3. `cli.py` - Add organization menu handlers

**Features**:
- Priorities & Tags (FR-008, FR-009)
- Search by keyword (FR-010)
- Filter by status, priority, tags (FR-011, FR-012, FR-013)
- Sort by date, priority, title (FR-014, FR-015, FR-016)

**Testing**:
- Unit tests for search/filter/sort logic
- Integration test for organization workflow
- Edge cases: empty search results, invalid priority, tag handling

**Deliverable**: Enhanced task organization capabilities

**Estimated Complexity**: Medium (multiple interrelated features)

### Phase 2.3: Advanced Level (Scheduling) - User Story P3

**Objective**: Implement due dates, reminders, recurring tasks

**Module Updates**:
1. `models.py` - Add due_date and recurrence fields
2. `services.py` - Add due date, reminder, recurrence functions
3. `cli.py` - Add advanced feature handlers

**Features**:
- Set Due Dates (FR-017, FR-018)
- Overdue Detection (FR-019)
- Console Reminders (FR-020)
- Recurring Tasks (FR-021, FR-022, FR-023, FR-024)

**Testing**:
- Unit tests for date/time handling
- Integration test for reminder system
- Edge cases: past due dates, recurring task regeneration

**Deliverable**: Complete feature set with advanced scheduling

**Estimated Complexity**: High (datetime logic, recurring patterns)

### Phase 2.4: Testing & Validation

**Objective**: Comprehensive testing across all features

**Test Coverage**:
1. Unit tests for all services.py functions (target: 100%)
2. Integration tests for complete user workflows
3. Edge case tests:
   - Empty task list operations
   - Invalid IDs (non-existent, negative, zero)
   - Boundary conditions (1000+ tasks, very long titles)
   - Special characters in input
   - Date/time edge cases (past dates, invalid formats)
   - Recurring task regeneration logic

**Testing Tools**:
- pytest for test execution
- Coverage reporting
- Manual CLI testing checklist

**Deliverable**: Verified, fully tested application

**Estimated Complexity**: Medium (systematic validation)

### Phase 2.5: Documentation & Finalization

**Objective**: Complete project documentation

**Deliverables**:
1. README.md - Setup instructions, usage guide, features list
2. CLAUDE.md - Claude Code workflow, agent usage, skill invocation
3. Update quickstart.md with final examples
4. Code comments and docstrings
5. Final PHR for project completion

**Estimated Complexity**: Low (documentation only)

---

## Architectural Decisions

### ADR-001: In-Memory Storage Structure

**Decision**: Use Python list as primary storage + dict for ID indexing

**Context**: Need fast task storage and retrieval without persistence

**Alternatives Considered**:
1. List only - O(n) lookups by ID
2. Dict only - Loses insertion order (before Python 3.7)
3. List + dict hybrid - O(1) lookups, maintains order

**Decision**: List + dict hybrid

**Rationale**:
- List maintains insertion/creation order
- Dict enables O(1) lookups by ID
- Minimal memory overhead
- Supports both "get all" and "get by ID" efficiently

**Consequences**:
- Must keep list and dict synchronized
- Slight memory overhead (two references per task)
- Deletion requires updating both structures

### ADR-002: Recurring Task Regeneration

**Decision**: Regenerate recurring tasks when marked complete (not on schedule)

**Context**: Advanced feature requires repeating tasks (daily, weekly, monthly)

**Alternatives Considered**:
1. Background scheduler checking continuously
2. Regenerate on next app start
3. Regenerate when task marked complete

**Decision**: Regenerate on completion

**Rationale**:
- No background threads needed (simpler)
- Aligns with assumption #2 in spec
- User sees immediate feedback
- Works for all recurrence patterns

**Consequences**:
- Task won't auto-regenerate if never completed
- User must mark complete to trigger next occurrence
- Simpler implementation, easier to test

### ADR-003: Console Reminder Implementation

**Decision**: Check due dates on menu display, show inline notifications

**Context**: Advanced feature requires reminders for due dates

**Alternatives Considered**:
1. Background thread polling for due tasks
2. OS-level notifications
3. Check on each menu display

**Decision**: Check on menu display

**Rationale**:
- Aligns with assumption #1 (reminders when app is active)
- No background threads or OS dependencies
- Simple implementation
- Adequate for console application

**Consequences**:
- Reminders only appear when user interacts with menu
- No notifications when app is idle
- Acceptable trade-off for console-only app

### ADR-004: UV Package Manager

**Decision**: Use UV for all dependency management

**Context**: Project requires modern Python tooling

**Alternatives Considered**:
1. pip + requirements.txt
2. Poetry
3. UV

**Decision**: UV

**Rationale**:
- Modern, fast dependency resolution
- Better lock file management
- Explicit user requirement
- Growing ecosystem adoption

**Consequences**:
- Users must install UV
- Potential learning curve for UV-unfamiliar users
- Benefits: faster installs, better dependency resolution

---

## Testing Strategy

### Unit Testing Scope

**Target Coverage**: 100% for services.py, 90%+ overall

**Test Files**:
- `tests/unit/test_models.py` - Task dataclass validation
- `tests/unit/test_storage.py` - Storage operations
- `tests/unit/test_services.py` - All business logic functions

**Key Test Scenarios**:
1. Task creation with valid/invalid inputs
2. CRUD operations success and failure paths
3. Search/filter/sort with various criteria
4. Due date and recurring task logic
5. ID generation and uniqueness
6. Validation functions

### Integration Testing Scope

**Target Coverage**: All user workflows from spec

**Test Files**:
- `tests/integration/test_cli.py` - End-to-end user journeys

**Key Test Scenarios**:
1. Complete CRUD cycle (create → view → update → complete → delete)
2. Organization workflow (create tasks → search → filter → sort)
3. Advanced workflow (create with due date → set recurrence → mark complete)

### Edge Case Testing Scope

**Test Files**:
- `tests/edge_cases/test_edge_cases.py`

**Key Test Scenarios**:
1. Empty task list operations
2. Invalid IDs (negative, zero, non-existent)
3. Boundary conditions (1000+ tasks, very long titles)
4. Special characters in titles and tags
5. Past due dates
6. Invalid date formats
7. Recurring task regeneration edge cases

---

## Risk Assessment

### High-Priority Risks

1. **Risk**: Recurring task date calculation errors
   - **Likelihood**: Medium
   - **Impact**: High (incorrect feature behavior)
   - **Mitigation**: Comprehensive datetime unit tests, manual validation

2. **Risk**: Performance degradation with 1000+ tasks
   - **Likelihood**: Low
   - **Impact**: Medium (violates success criteria SC-009)
   - **Mitigation**: Performance testing, optimize if needed

3. **Risk**: Complex CLI menu navigation confusion
   - **Likelihood**: Medium
   - **Impact**: Medium (poor UX, violates SC-010)
   - **Mitigation**: User testing, clear menu labels, help text

### Medium-Priority Risks

4. **Risk**: Date format parsing inconsistencies
   - **Likelihood**: Medium
   - **Impact**: Low (validation will catch most issues)
   - **Mitigation**: Strict format validation, clear error messages

5. **Risk**: UV package manager unfamiliarity
   - **Likelihood**: High
   - **Impact**: Low (documentation can address)
   - **Mitigation**: Comprehensive setup instructions in README

---

## Success Metrics

Based on spec Success Criteria:

1. **SC-001**: Task creation < 5 seconds ✓ (Expected: < 1 second)
2. **SC-002**: Full CRUD cycle < 2 minutes ✓ (Expected: < 1 minute)
3. **SC-003**: Search/filter < 1 second for 1000 tasks ✓ (Target: < 0.5 sec)
4. **SC-004**: 95% input validation ✓ (Target: 100%)
5. **SC-005**: Intuitive priority/tag usage ✓ (Manual validation)
6. **SC-006**: 100% recurring task accuracy ✓ (Automated testing)
7. **SC-007**: Reminders display correctly ✓ (Integration testing)
8. **SC-008**: All 10 features implemented ✓ (Feature checklist)
9. **SC-009**: 100+ tasks without degradation ✓ (Performance testing)
10. **SC-010**: Intuitive navigation ✓ (Manual usability testing)

---

## Timeline Estimate

**Phase 2.1 (Basic)**: 2-3 implementation sessions
**Phase 2.2 (Intermediate)**: 2-3 implementation sessions
**Phase 2.3 (Advanced)**: 3-4 implementation sessions
**Phase 2.4 (Testing)**: 1-2 validation sessions
**Phase 2.5 (Documentation)**: 1 session

**Total Estimated**: 9-13 implementation sessions

---

## Next Steps

1. ✅ Complete Phase 0: Generate `research.md` with technical research
2. ✅ Complete Phase 1: Generate design artifacts:
   - `data-model.md`
   - `contracts/` (module APIs)
   - `quickstart.md`
3. ⏭️ Run `/sp.tasks` to generate `tasks.md` with actionable implementation tasks
4. ⏭️ Begin implementation following task order

**Current Status**: Plan complete, ready for Phase 0 research

**Constitution Re-Check**: Will be performed after Phase 1 design artifacts are complete
