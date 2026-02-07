# Data Model: TASKAPP Local Workspace

**Feature**: 1-ui-redesign-routing
**Created**: 2026-01-16

## Entity: Task

### Properties
- **id**: string (required)
  - Unique identifier for the task
  - Generated as UUID when created
  - Immutable after creation

- **title**: string (required)
  - Task title or description
  - Length: 1-100 characters
  - Cannot be empty or whitespace only

- **description**: string (optional)
  - Detailed task description
  - Length: max 5000 characters
  - Can be null or empty string

- **completed**: boolean (required)
  - Completion status of the task
  - Default: false
  - Values: true (completed) or false (pending)

- **priority**: string (optional)
  - Task priority level
  - Values: "high", "medium", "low"
  - Can be null if not specified

- **tags**: string[] (optional)
  - Array of tags associated with the task
  - Each tag: 1-30 characters
  - Default: empty array

- **dueDate**: string (optional)
  - Due date for the task in ISO 8601 format
  - Format: YYYY-MM-DDTHH:mm:ss.sssZ
  - Can be null if not specified

- **recurrence**: string (optional)
  - Recurrence pattern for the task
  - Values: "none", "daily", "weekly"
  - Default: "none"

- **createdAt**: string (required)
  - Creation timestamp in ISO 8601 format
  - Format: YYYY-MM-DDTHH:mm:ss.sssZ
  - Set automatically on creation

- **updatedAt**: string (required)
  - Last update timestamp in ISO 8601 format
  - Format: YYYY-MM-DDTHH:mm:ss.sssZ
  - Updated automatically on modification

### Validation Rules
1. Title must be 1-100 characters
2. Description must be 0-5000 characters if provided
3. Priority must be one of "high", "medium", "low" if provided
4. Each tag must be 1-30 characters
5. Due date must be in valid ISO 8601 format if provided
6. Recurrence must be one of "none", "daily", "weekly" if provided

### State Transitions
- `completed` can toggle between true/false
- `updatedAt` updates on any property change
- Other properties can be modified individually

## Entity: LocalWorkspace

### Properties
- **id**: string (required)
  - Unique identifier for the workspace
  - Generated as UUID when created

- **tasks**: Task[] (required)
  - Array of tasks in the workspace
  - Default: empty array

- **lastModified**: string (required)
  - Timestamp of last modification in ISO 8601 format
  - Format: YYYY-MM-DDTHH:mm:ss.sssZ
  - Updated automatically when tasks are modified

- **settings**: object (optional)
  - User-specific settings for the workspace
  - Properties:
    - theme: "light" | "dark" (default: "dark")
    - notificationsEnabled: boolean (default: true)
    - defaultPriority: "high" | "medium" | "low" (default: "medium")

### Validation Rules
1. Tasks array must contain valid Task objects
2. Last modified must be in valid ISO 8601 format
3. Settings properties must conform to their type definitions

## Entity: NavigationItem

### Properties
- **id**: string (required)
  - Unique identifier for the navigation item
  - Examples: "dashboard", "tasks", "create-task"

- **label**: string (required)
  - Display text for the navigation item
  - Examples: "Dashboard", "Tasks", "Create Task"

- **href**: string (required)
  - Route path for the navigation item
  - Examples: "/dashboard", "/tasks", "/tasks/create"

- **icon**: string (optional)
  - Icon identifier or component name
  - Examples: "dashboard-icon", "task-icon"

- **isActive**: boolean (required)
  - Whether this navigation item is currently active
  - Default: false

### Validation Rules
1. Label must be 1-50 characters
2. Href must be a valid relative path starting with "/"
3. Active state must be boolean

## Relationships

### Task ↔ LocalWorkspace
- One LocalWorkspace contains many Tasks
- Tasks exist within the context of a LocalWorkspace
- When LocalWorkspace is cleared, all contained Tasks are removed

## Data Access Patterns

### Task Operations
1. **Create**: Add new Task to LocalWorkspace.tasks array
2. **Read**: Retrieve Task by ID from LocalWorkspace.tasks
3. **Update**: Modify properties of existing Task in LocalWorkspace.tasks
4. **Delete**: Remove Task from LocalWorkspace.tasks by ID
5. **Filter**: Get subset of tasks by criteria (completed, priority, etc.)
6. **Sort**: Order tasks by various properties (dueDate, createdAt, priority)

### LocalWorkspace Operations
1. **Initialize**: Create new LocalWorkspace with empty tasks array
2. **Load**: Restore LocalWorkspace from persistent storage
3. **Save**: Persist LocalWorkspace to storage
4. **Clear**: Remove all tasks from LocalWorkspace
5. **Export**: Serialize LocalWorkspace for backup/sharing
6. **Import**: Deserialize and merge tasks into LocalWorkspace

## Storage Schema

### Local Storage Key Structure
```
TASKAPP_WORKSPACE = {
  id: string,
  tasks: Task[],
  lastModified: string,
  settings: {
    theme: "light" | "dark",
    notificationsEnabled: boolean,
    defaultPriority: "high" | "medium" | "low"
  }
}
```

### Migration Considerations
- Previous versions may have stored data differently
- Migration logic needed for backward compatibility
- Graceful degradation for unsupported storage mechanisms