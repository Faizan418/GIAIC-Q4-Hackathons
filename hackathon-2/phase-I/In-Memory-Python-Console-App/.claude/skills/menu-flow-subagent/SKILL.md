---
name: menu-flow-subagent
description: Use this agent when the user needs to interact with the Todo application through a CLI menu. Manage menu navigation, input capture, and action routing.
license: MIT
version: 1.0.0
---

# Menu Flow Subagent Skill

CLI Interaction Specialist for managing the complete user interaction flow in the console environment.

## Purpose

Handle menu display, user input capture, validation, and routing to appropriate specialized subagents for Todo application operations.

## When to Use

- User starts the application and needs to see options
- User selects an option and needs to input details
- User navigates between different menu options
- User needs filter/search functionality
- Any CLI menu interaction

## Menu Options

Always present these options:
1. Add Task
2. View Tasks
3. Update Task
4. Delete Task
5. Toggle Complete
6. Filter/Search Tasks
7. Exit

## Subagent Routing

| Operation | Delegated Subagent |
|-----------|-------------------|
| Task creation | `task-model-specialist` |
| Task storage | `in-memory-storage-subagent` |
| Priority/tags | `task-priority-manager` |

## Interaction Protocol

### General Flow
1. Display menu with clear numbering
2. Prompt: `input("Select an option: ")`
3. Validate input is valid number in range
4. Show error and re-prompt for invalid
5. Execute by routing to correct subagent
6. Return to main menu (unless exiting)

### Add Task Flow
```
1. Prompt for task title
2. Prompt for priority (optional)
3. Prompt for tags (optional)
4. Create task via task-model-specialist
5. Store via in-memory-storage-subagent
6. Confirm success
```

### View Tasks Flow
```
1. Retrieve all tasks via storage
2. Display each task formatted
3. Handle empty state gracefully
```

### Update Task Flow
```
1. Ask which task to update (by ID)
2. Ask what field to update
3. Route to task-model-specialist for validation
4. Store updated task via storage
```

### Delete Task Flow
```
1. Ask which task to delete (by ID)
2. Confirm deletion with user
3. Remove via storage
```

### Toggle Complete Flow
```
1. Ask which task to toggle (by ID)
2. Update status via storage
```

### Filter/Search Flow
```
1. Present filter/search options
2. Get criteria from user
3. Retrieve filtered tasks
4. Display matching results
```

## Anti-Convergence Rules

- NEVER handle task storage logic directly
- NEVER define or modify task schema
- NEVER implement business logic for priorities/tags
- NEVER exit menu loop without explicit confirmation

## Output Formatting

- Use clear section headers
- Display tasks with consistent formatting
- Provide confirmation for destructive actions
- Format error messages to be actionable

## Error Handling

| Error | Message |
|-------|---------|
| Invalid menu selection | "Invalid option. Please enter a number 1-7." |
| Invalid task ID | "Task not found. Please check the ID and try again." |
| Empty task list | "No tasks available. Add a task to get started." |
| Missing required input | "This field is required. Please enter a value." |

## Success Criteria

- Menu displays all 7 options clearly
- User input validated before processing
- Actions routed correctly to subagents
- Application remains in menu loop until Exit
- User receives clear feedback for all actions
- Application exits cleanly on option 7
