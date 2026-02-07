# Data Model: TASKAPP Dashboard Features

**Feature**: 4-dashboard-features
**Date**: 2026-01-15
**Status**: Complete

## 1. Core Entities

### 1.1 Task Entity
```typescript
interface Task {
  // Primary identifier
  id: string;                    // Unique identifier (UUID v4 format)

  // Core properties
  title: string;                 // Task title (required, 1-200 chars)
  description?: string;          // Task description (optional, max 1000 chars)
  completed: boolean;            // Completion status (default: false)

  // Organization properties
  priority: 'low' | 'medium' | 'high';  // Priority level (default: 'medium')
  tags: string[];                // Task tags (max 10 tags, 1-30 chars each)
  dueDate?: string;              // Due date in ISO 8601 format (YYYY-MM-DDTHH:mm:ss.sssZ)

  // Recurrence (advanced feature)
  recurrence?: {
    type: 'none' | 'daily' | 'weekly' | 'monthly';  // Recurrence interval
    interval: number;           // Interval multiplier (e.g., every 2 weeks)
    endDate?: string;           // End date for recurrence in ISO 8601 format
    occurrences?: number;       // Max number of occurrences (alternative to endDate)
  };

  // Metadata
  createdAt: string;             // Creation timestamp (ISO 8601 format)
  updatedAt: string;             // Last update timestamp (ISO 8601 format)
  completedAt?: string;          // Completion timestamp (ISO 8601 format, only when completed=true)
}
```

**Validation Rules**:
- `id`: Required, UUID v4 format, unique
- `title`: Required, 1-200 characters, trimmed
- `description`: Optional, 0-1000 characters, trimmed
- `completed`: Required, boolean
- `priority`: Required, one of 'low', 'medium', 'high'
- `tags`: Optional, array of strings, max 10 tags, each 1-30 characters, trimmed
- `dueDate`: Optional, valid ISO 8601 date string
- `recurrence`: Optional object with valid recurrence configuration
- `createdAt`: Required, valid ISO 8601 timestamp
- `updatedAt`: Required, valid ISO 8601 timestamp, updated on every change
- `completedAt`: Optional, valid ISO 8601 timestamp, only present when completed=true

**State Transitions**:
- `completed: false` → `completed: true` (when task marked as complete)
- `completed: true` → `completed: false` (when task marked as incomplete)
- `dueDate` can be added/changed/removed at any time
- `recurrence` can be added/changed/removed at any time

### 1.2 User Preferences Entity
```typescript
interface UserPreferences {
  userId: string;                // Reference to user (for future auth integration)

  // Theme settings
  theme: 'light' | 'dark' | 'system';  // Selected theme (default: 'system')

  // Notification settings
  notifications: {
    enabled: boolean;            // Whether notifications are enabled (default: true)
    dueTimeHours: number;        // Hours before due time to notify (default: 24)
    deadlineReminders: boolean;  // Whether to send deadline reminders (default: true)
  };

  // Dashboard settings
  dashboard: {
    defaultView: 'list' | 'grid';      // Default task view (default: 'list')
    showCompleted: boolean;             // Whether to show completed tasks (default: true)
    defaultSort: 'dueDate' | 'priority' | 'title' | 'created';  // Default sort (default: 'dueDate')
    defaultSortOrder: 'asc' | 'desc';  // Default sort order (default: 'asc')
  };

  // Metadata
  createdAt: string;             // Creation timestamp (ISO 8601 format)
  updatedAt: string;             // Last update timestamp (ISO 8601 format)
}
```

**Validation Rules**:
- `userId`: Required, valid UUID format
- `theme`: Required, one of 'light', 'dark', 'system'
- `notifications.enabled`: Required, boolean
- `notifications.dueTimeHours`: Required, number between 1-168 (1 week)
- `notifications.deadlineReminders`: Required, boolean
- `dashboard.defaultView`: Required, one of 'list', 'grid'
- `dashboard.showCompleted`: Required, boolean
- `dashboard.defaultSort`: Required, one of 'dueDate', 'priority', 'title', 'created'
- `dashboard.defaultSortOrder`: Required, one of 'asc', 'desc'
- `createdAt`: Required, valid ISO 8601 timestamp
- `updatedAt`: Required, valid ISO 8601 timestamp

## 2. Relationships

### 2.1 Task Relationships
- One User (future) → Many Tasks (via userId reference)
- One Task → Zero or One Recurrence Rule (embedded in task)
- One Task → Many Tags (embedded in task)

### 2.2 User Preferences Relationships
- One User (future) → One UserPreferences (1:1 relationship)

## 3. Indexes & Performance Considerations

### 3.1 Client-Side Indexes (for efficient querying in localStorage)
```typescript
// Indexes for efficient filtering and sorting
indexes: {
  // For filtering by completion status
  byStatus: Map<boolean, Task[]>;  // completed=true/false

  // For filtering by priority
  byPriority: Map<string, Task[]>; // priority='high'/'medium'/'low'

  // For filtering by due date (grouped by date ranges)
  byDueDate: Map<string, Task[]>;  // dueDate ranges: 'overdue', 'today', 'thisWeek', 'thisMonth', 'later'

  // For searching by title/description/tags
  searchIndex: Map<string, Set<string>>; // word → Set of taskIds containing that word

  // For sorting operations
  sortedViews: {
    byDueDate: Task[];      // Sorted by due date ascending
    byPriority: Task[];     // Sorted by priority (high→medium→low)
    byTitle: Task[];        // Sorted by title alphabetically
    byCreated: Task[];      // Sorted by creation date descending
  }
}
```

## 4. Data Storage Schema (localStorage)

### 4.1 Storage Keys
```typescript
storageKeys: {
  'tasks': string;           // JSON string of Task[] array
  'userPreferences': string; // JSON string of UserPreferences object
  'lastSync': string;        // Timestamp of last sync (for future backend sync)
  'version': string;         // Schema version for migration purposes
}
```

### 4.2 Data Format
```typescript
// localStorage.getItem('tasks') returns:
'[{...task1...}, {...task2...}, ...]'

// localStorage.getItem('userPreferences') returns:
'{...userPreferencesObject...}'

// localStorage.getItem('lastSync') returns:
'2026-01-15T10:30:00.000Z'

// localStorage.getItem('version') returns:
'1.0.0'
```

## 5. Validation & Business Rules

### 5.1 Task Validation
- Title must be 1-200 characters after trimming whitespace
- Description must be 0-1000 characters after trimming whitespace
- Due date must be a valid future date (for new tasks)
- Priority must be one of the allowed values
- Tags array must not exceed 10 items
- Each tag must be 1-30 characters after trimming
- Recurrence settings must be consistent (endDate vs occurrences)
- Creation and update timestamps are automatically managed

### 5.2 Recurrence Validation
- When `type` is 'none', no other recurrence fields allowed
- When `type` is not 'none', `interval` must be >= 1
- `endDate` and `occurrences` cannot both be set simultaneously
- `endDate` must be in the future if specified
- `occurrences` must be >= 1 if specified

### 5.3 Data Integrity Rules
- All timestamps must be valid ISO 8601 format
- All UUIDs must follow valid UUID v4 format
- Completed tasks cannot have past due dates (business rule)
- Tasks with recurrence cannot be marked as completed (they generate new instances)

## 6. Migration Considerations

### 6.1 Schema Evolution
```typescript
// Version 1.0.0 (current): Basic task structure
// Version 1.1.0: Add recurrence field
// Version 1.2.0: Add completedAt field
// Version 2.0.0: Future backend migration (different storage mechanism)

migrationPaths: [
  {
    from: '1.0.0',
    to: '1.1.0',
    changes: ['Add recurrence field with default value'],
    transformer: (oldTask: any) => ({
      ...oldTask,
      recurrence: { type: 'none' }
    })
  },
  {
    from: '1.1.0',
    to: '1.2.0',
    changes: ['Add completedAt field with default null'],
    transformer: (oldTask: any) => ({
      ...oldTask,
      completedAt: oldTask.completed ? oldTask.updatedAt : undefined
    })
  }
]
```

## 7. API Contract (Future Backend Integration)

### 7.1 Task Endpoints (for future reference)
```typescript
// GET /api/tasks
interface GetTasksRequest {
  userId: string;
  filters?: {
    status?: 'all' | 'completed' | 'pending';
    priority?: 'all' | 'high' | 'medium' | 'low';
    dueDateRange?: 'all' | 'overdue' | 'today' | 'week' | 'month';
    tags?: string[];
  };
  sort?: {
    by: 'dueDate' | 'priority' | 'title' | 'created';
    order: 'asc' | 'desc';
  };
  search?: string;
  pagination?: {
    page: number;
    limit: number;
  };
}

interface GetTasksResponse {
  success: boolean;
  tasks: Task[];
  pagination: {
    page: number;
    limit: number;
    total: number;
    totalPages: number;
  };
  error?: string;
}

// POST /api/tasks
interface CreateTaskRequest {
  userId: string;
  task: Omit<Task, 'id' | 'createdAt' | 'updatedAt' | 'completedAt'>;
}

interface CreateTaskResponse {
  success: boolean;
  task: Task;
  error?: string;
}

// PUT /api/tasks/{taskId}
interface UpdateTaskRequest {
  userId: string;
  taskId: string;
  task: Partial<Omit<Task, 'id' | 'createdAt'>>;
}

interface UpdateTaskResponse {
  success: boolean;
  task: Task;
  error?: string;
}

// DELETE /api/tasks/{taskId}
interface DeleteTaskRequest {
  userId: string;
  taskId: string;
}

interface DeleteTaskResponse {
  success: boolean;
  error?: string;
}
```

## 8. Local Storage Limitations & Error Handling

### 8.1 Storage Quota Limits
- Typical localStorage limit: 5-10MB per origin
- Estimated per-task size: ~500 bytes (conservative estimate)
- Safe maximum tasks: ~10,000 tasks (with significant buffer)
- Monitoring: Track storage usage and warn when approaching limits

### 8.2 Error Handling Strategies
```typescript
storageErrorStrategies: [
  {
    error: 'QuotaExceededError',
    strategy: 'Reduce data size by removing old completed tasks or warn user'
  },
  {
    error: 'SecurityError',
    strategy: 'Graceful degradation - allow app to function without persistence'
  },
  {
    error: 'SyntaxError (JSON parsing)',
    strategy: 'Fallback to empty state and warn user about corrupted data'
  }
]
```

## 9. Performance Considerations

### 9.1 Data Size Optimization
- Store only necessary fields (avoid redundant computed values)
- Use efficient data structures (Sets for lookups, Maps for indexing)
- Implement compression for large datasets if needed
- Batch updates to reduce localStorage write operations

### 9.2 Query Performance
- Maintain pre-computed indexes for common queries
- Use efficient algorithms for sorting/filtering operations
- Implement virtualization for large result sets
- Cache expensive computations where appropriate