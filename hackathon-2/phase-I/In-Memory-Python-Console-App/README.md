# Todo In-Memory Python Console App

A fully functional in-memory Python console application for managing tasks, built using Claude Code and Spec-Driven Development.

## Features

### Basic Level (5 Features)
- ✅ **Add Task** - Create new todo items with title, priority, and tags
- ✅ **Delete Task** - Remove tasks from the list with confirmation
- ✅ **Update Task** - Modify task title, priority, and tags
- ✅ **View Task List** - Display all tasks with formatting
- ✅ **Mark as Complete** - Toggle task completion status

### Intermediate Level (3 Features)
- ✅ **Priorities & Tags** - Assign High/Medium/Low priorities and categorize with tags
- ✅ **Search & Filter** - Search by keyword, filter by status, priority, or tag
- ✅ **Sort Tasks** - Reorder by due date, priority, or alphabetically

### Advanced Level (2 Features)
- ✅ **Recurring Tasks** - Auto-regenerate tasks (Daily/Weekly/Monthly)
- ✅ **Due Dates & Reminders** - Set deadlines and get console notifications

## Requirements

- Python 3.13+
- UV package manager (recommended)

## Installation

### Install UV Package Manager

**Windows (PowerShell)**:
```powershell
irm https://astral.sh/uv/install.ps1 | iex
```

**macOS/Linux**:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Install Dependencies

```bash
uv sync
```

Or with optional dependencies:
```bash
uv sync --extra dev
```

## Running the Application

### With UV:
```bash
uv run python -m src.main
```

### With Standard Python:
```bash
python -m src.main
```

## Usage Guide

### Basic Operations

1. **Add a Task**
   - Select option [1] from the main menu
   - Enter task title
   - Choose priority (High/Medium/Low)
   - Add tags (optional, comma-separated)

2. **View All Tasks**
   - Select option [2]
   - See complete task list with status, priority, tags, and due dates

3. **Update a Task**
   - Select option [3]
   - Enter task ID
   - Update title, priority, or tags

4. **Delete a Task**
   - Select option [4]
   - Enter task ID
   - Confirm deletion (yes/no)

5. **Toggle Completion**
   - Select option [5]
   - Enter task ID
   - Task completion status toggles

### Organization Features

6. **Search Tasks**
   - Select option [6]
   - Enter search keyword
   - See matching tasks (case-insensitive)

7. **Filter Tasks**
   - Select option [7]
   - Choose filter type:
     - Completed/Pending tasks
     - High/Medium/Low priority
     - By specific tag

8. **Sort Tasks**
   - Select option [8]
   - Choose sort criteria:
     - Priority (High to Low)
     - Date (Newest/Oldest First)
     - Title (A-Z or Z-A)

### Advanced Features

9. **Set Due Date**
   - Select option [9]
   - Enter task ID
   - Enter due date (format: YYYY-MM-DD HH:MM)
   - Overdue tasks show ⚠️ OVERDUE marker

10. **Set Recurring Task**
    - Select option [10]
    - Enter task ID
    - Choose frequency (Daily/Weekly/Monthly)
    - Enter start date
    - Task auto-regenerates when marked complete

### Reminders

- Console reminders display automatically on menu screen
- Shows tasks due within 5 minutes
- Only appears when application is running

## Project Structure

```
src/
├── models.py      # Task and RecurrencePattern dataclasses
├── storage.py     # In-memory storage (list + dict hybrid)
├── services.py    # Business logic and CRUD operations
├── cli.py         # User interface and menu handling
└── main.py        # Application entry point

tests/
├── unit/          # Unit tests for services
├── integration/   # End-to-end workflow tests
└── edge_cases/    # Boundary condition tests

specs/001-todo-app/
├── spec.md        # Feature specification
├── plan.md        # Implementation plan
├── research.md    # Technical research
└── tasks.md       # Task breakdown

.specify/
└── memory/
    └── constitution.md  # Project principles (v1.0.0)
```

## Technical Details

- **Storage**: In-memory using Python lists and dictionaries (no persistence)
- **Data Resets**: All task data is ephemeral and resets on program exit
- **Platform**: Cross-platform (Windows, macOS, Linux)
- **Dependencies**: Standard library only (optional: colorama for enhanced output)

## Development Workflow

This project follows Spec-Driven Development (SDD):

1. **Specification** (`specs/001-todo-app/spec.md`) - Defined requirements
2. **Planning** (`specs/001-todo-app/plan.md`) - Architecture and design decisions
3. **Tasks** (`specs/001-todo-app/tasks.md`) - 80 actionable implementation tasks
4. **Implementation** - Executed via Claude Code automation
5. **Validation** - Tested against success criteria

See **CLAUDE.md** for Claude Code workflow details.

## Constitution

Project governed by principles defined in `.specify/memory/constitution.md` (v1.0.0):

- Spec-Driven Development
- In-Memory Architecture
- Modular Python Structure
- Feature Completeness
- Testing & Quality
- Console-First UX
- Reproducibility

## Testing

Run tests with:
```bash
uv run pytest tests/
```

Or with coverage:
```bash
uv run pytest tests/ --cov=src --cov-report=html
```

## License

MIT

## Author

Built with Claude Code using Spec-Kit Plus
