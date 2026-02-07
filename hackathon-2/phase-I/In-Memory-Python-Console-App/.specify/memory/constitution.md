<!--
SYNC IMPACT REPORT
==================
Version Change: Initial (undefined) → 1.0.0
Type: MINOR - Initial constitution establishment

Modified Principles:
- NEW: I. Spec-Driven Development
- NEW: II. In-Memory Architecture
- NEW: III. Modular Python Structure
- NEW: IV. Feature Completeness
- NEW: V. Testing & Quality
- NEW: VI. Console-First UX
- NEW: VII. Reproducibility

Added Sections:
- Core Principles (7 principles)
- Technology Stack & Constraints
- Development Workflow
- Governance

Removed Sections: None (initial version)

Templates Requiring Updates:
- ✅ plan-template.md - Constitution Check section already supports this structure
- ✅ spec-template.md - User scenarios and requirements align with feature completeness principle
- ✅ tasks-template.md - Task organization supports modular structure and testing principle
- ⚠️ README.md - PENDING (does not exist yet, should be created)
- ⚠️ CLAUDE.md - PENDING (should be updated to reference this constitution)

Follow-up TODOs:
- Create README.md with project setup instructions
- Update CLAUDE.md to reference constitution version and location
- Ensure all command files reference this constitution for validation
-->

# Todo In-Memory Python Console App Constitution

## Core Principles

### I. Spec-Driven Development

All development MUST follow the Spec-Driven Development (SDD) workflow using Claude Code and Spec-Kit Plus:

- Feature specifications created BEFORE implementation (spec.md)
- Architecture plans documented with design decisions (plan.md)
- Tasks broken down into testable, actionable items (tasks.md)
- All specifications stored in `/specs` folder with complete history tracking
- Prompt History Records (PHRs) created for every user interaction
- Architectural Decision Records (ADRs) required for significant design choices

**Rationale**: Spec-driven development ensures alignment, reduces rework, maintains quality standards, and provides comprehensive documentation for project understanding and maintenance.

### II. In-Memory Architecture

Task data MUST be stored entirely in memory using Python lists and dictionaries:

- NO file persistence (no JSON, CSV, or text file storage)
- NO database connections (no SQLite, PostgreSQL, or any DB)
- Data structure: lists/dictionaries managed in core storage module
- All task data is ephemeral and resets on program restart
- Clear separation between storage layer and business logic

**Rationale**: In-memory architecture simplifies implementation, eliminates external dependencies, ensures fast operations, and meets the explicit project constraint of ephemeral data storage suitable for learning and prototyping.

### III. Modular Python Structure

Code MUST follow proper Python project structure with clear separation of concerns:

- **models.py** - Task data model definitions (dataclasses)
- **storage.py** - In-memory storage management (lists/dicts)
- **services.py** - Business logic and CRUD operations
- **cli.py** - User interface and menu handling
- **main.py** - Application entry point
- **tests/** - Test modules organized by layer

Each module has single responsibility; business logic separated from UI; testable components with minimal coupling.

**Rationale**: Modular structure enables independent testing, supports future enhancements, follows SOLID principles, improves code organization and maintainability, and allows parallel development of different layers.

### IV. Feature Completeness

ALL required features across three levels MUST be fully implemented and working:

**Basic Level (5 features - NON-NEGOTIABLE)**:
- Add Task - Create new todo items with title, priority, tags
- Delete Task - Remove tasks from the list by ID
- Update Task - Modify existing task details (title, priority, tags)
- View Task List - Display all tasks with formatting
- Mark as Complete - Toggle task completion status

**Intermediate Level (3 features - NON-NEGOTIABLE)**:
- Priorities & Tags/Categories - Assign levels (High/Medium/Low) and labels (Work/Home/Personal)
- Search & Filter - Search by keyword; filter by status, priority, or date
- Sort Tasks - Reorder by due date, priority, or alphabetically

**Advanced Level (2 features - NON-NEGOTIABLE)**:
- Recurring Tasks - Auto-reschedule repeating tasks (e.g., daily, weekly, monthly)
- Due Dates & Time Reminders - Set deadlines with date/time; display console notifications

**Rationale**: Complete feature implementation across all three levels is the explicit success criterion for project submission. Partial implementation is unacceptable and fails project requirements.

### V. Testing & Quality

Core functionality and edge cases MUST be tested comprehensively:

- Unit tests for all business logic functions (services.py)
- Integration tests for user workflows (CLI interactions)
- Edge case coverage (empty inputs, invalid data, boundary conditions)
- Test organization mirrors module structure (tests/test_services.py, etc.)
- All tests must pass before feature is considered complete
- Test-driven approach encouraged (write tests, verify failure, then implement)

**Rationale**: Testing ensures correctness, catches regressions early, validates edge cases, provides executable documentation of expected behavior, and maintains code quality throughout development.

### VI. Console-First UX

User interaction MUST be clear, intuitive, and console-appropriate:

- CLI-only interface (no GUI, no web frontend)
- Numbered menu options with clear descriptions
- Input validation with helpful error messages
- Confirmation prompts for destructive actions (delete)
- Readable task formatting (tables, status indicators)
- Clean output without clutter

**Rationale**: Console-first design meets project constraints, ensures accessibility, supports automation/scripting, maintains simplicity, and provides immediate visual feedback appropriate for terminal environments.

### VII. Reproducibility

All features MUST produce consistent, repeatable results:

- Deterministic behavior for all operations
- Clear state management (what's in memory at any time)
- Predictable ID generation (sequential, unique)
- Consistent sorting and filtering logic
- No hidden state or side effects
- All operations reversible or clearly documented

**Rationale**: Reproducibility ensures testability, enables debugging, supports validation across test runs, maintains user trust, and allows for reliable demonstration of features.

## Technology Stack & Constraints

**Language & Version**: Python 3.13+ (REQUIRED)

**Core Dependencies**:
- Standard library only for core functionality
- Optional: colorama for enhanced console output
- Optional: pytest for testing framework

**Storage**: In-memory lists and dictionaries (NO persistence)

**Interface**: Command-line only (NO GUI, NO web)

**Platform**: Cross-platform (Windows, macOS, Linux)

**Constraints**:
- All task data ephemeral (resets on program exit)
- No external services or APIs
- No authentication/authorization (single-user)
- No concurrent user support
- No network operations

## Development Workflow

**Phase 1: Specification**
1. Create feature specification (spec.md)
2. Define user scenarios with acceptance criteria
3. Identify requirements and success criteria
4. Get user approval before proceeding

**Phase 2: Architecture Planning**
1. Document technical approach (plan.md)
2. Design module structure and APIs
3. Make architectural decisions (record ADRs if significant)
4. Define test strategy

**Phase 3: Task Breakdown**
1. Break plan into actionable tasks (tasks.md)
2. Organize tasks by user story priority
3. Identify parallel vs. sequential tasks
4. Define acceptance criteria per task

**Phase 4: Implementation**
1. Implement tasks in dependency order
2. Test each component as implemented
3. Validate against acceptance criteria
4. Create PHRs for each implementation session

**Phase 5: Integration & Testing**
1. Run full test suite
2. Validate all features work end-to-end
3. Test edge cases and error handling
4. Document any known issues

**No Manual Coding**: All code generation MUST be done through Claude Code automation workflow - no manual editing outside of Claude Code environment.

## Governance

This constitution supersedes all other development practices and guidelines.

**Amendment Process**:
- Amendments require documented justification
- Version must be incremented (MAJOR.MINOR.PATCH)
- All dependent templates must be updated for consistency
- Sync Impact Report required for all changes

**Versioning Policy**:
- **MAJOR**: Backward incompatible governance/principle removals or redefinitions
- **MINOR**: New principle/section added or materially expanded guidance
- **PATCH**: Clarifications, wording, typo fixes, non-semantic refinements

**Compliance Review**:
- All implementations must verify compliance with this constitution
- Constitution Check gate in plan.md validates adherence
- Complexity must be justified if principles are violated
- Regular audits to ensure ongoing alignment

**Documentation**:
- Constitution version referenced in all specification artifacts
- CLAUDE.md provides runtime guidance referencing this constitution
- README.md includes reference to constitution location

**Enforcement**:
- All code reviews verify principle compliance
- Test suites validate feature completeness
- Specification artifacts checked for completeness
- PHR creation mandatory for all user interactions

**Version**: 1.0.0 | **Ratified**: 2025-12-29 | **Last Amended**: 2025-12-29
