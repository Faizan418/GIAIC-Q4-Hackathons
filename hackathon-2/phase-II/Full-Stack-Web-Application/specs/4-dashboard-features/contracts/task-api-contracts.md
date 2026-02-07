# API Contracts: TASKAPP Dashboard Features

**Feature**: 4-dashboard-features
**Date**: 2026-01-15
**Status**: Specification

## Overview

This document defines the API contracts for the TASKAPP dashboard features, focusing on client-side persistence using localStorage and the anticipated backend API contracts for future migration.

## 1. Task Management Contracts

### 1.1 Get Tasks
**Client Implementation**: Retrieve all tasks from localStorage

```typescript
// Input
interface GetTasksRequest {
  userId?: string; // For future backend implementation
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

// Output
interface GetTasksResponse {
  success: true;
  tasks: Task[];
  pagination?: {
    page: number;
    limit: number;
    total: number;
    totalPages: number;
  };
  error?: string;
}
```

**Client Implementation Details**:
- Reads from localStorage key: `taskapp-tasks`
- Applies client-side filtering, sorting, and search
- Returns tasks array with applied filters
- No network request - synchronous operation

### 1.2 Create Task
**Client Implementation**: Add new task to localStorage

```typescript
// Input
interface CreateTaskRequest {
  userId?: string; // For future backend implementation
  task: {
    title: string;
    description?: string;
    priority: 'low' | 'medium' | 'high';
    tags: string[];
    dueDate?: string;
    recurrence?: {
      type: 'none' | 'daily' | 'weekly' | 'monthly';
      interval: number;
      endDate?: string;
      occurrences?: number;
    };
  };
}

// Output
interface CreateTaskResponse {
  success: true;
  task: Task;
  error?: string;
}
```

**Client Implementation Details**:
- Generates unique ID using crypto.randomUUID()
- Sets createdAt and updatedAt to current timestamp
- Sets completed to false by default
- Saves to localStorage by appending to tasks array
- Updates localStorage atomically

### 1.3 Update Task
**Client Implementation**: Update existing task in localStorage

```typescript
// Input
interface UpdateTaskRequest {
  userId?: string; // For future backend implementation
  taskId: string;
  updates: {
    title?: string;
    description?: string;
    completed?: boolean;
    priority?: 'low' | 'medium' | 'high';
    tags?: string[];
    dueDate?: string;
    recurrence?: {
      type: 'none' | 'daily' | 'weekly' | 'monthly';
      interval: number;
      endDate?: string;
      occurrences?: number;
    };
  };
}

// Output
interface UpdateTaskResponse {
  success: true;
  task: Task;
  error?: string;
}
```

**Client Implementation Details**:
- Finds task by ID in localStorage tasks array
- Updates specified fields
- Updates updatedAt timestamp
- Saves updated tasks array to localStorage
- Maintains data integrity during update

### 1.4 Delete Task
**Client Implementation**: Remove task from localStorage

```typescript
// Input
interface DeleteTaskRequest {
  userId?: string; // For future backend implementation
  taskId: string;
}

// Output
interface DeleteTaskResponse {
  success: true;
  error?: string;
}
```

**Client Implementation Details**:
- Removes task with matching ID from localStorage tasks array
- Updates localStorage with new tasks array
- Returns success/failure status

### 1.5 Toggle Task Completion
**Client Implementation**: Toggle completion status of task

```typescript
// Input
interface ToggleTaskCompletionRequest {
  userId?: string; // For future backend implementation
  taskId: string;
  completed: boolean;
}

// Output
interface ToggleTaskCompletionResponse {
  success: true;
  task: Task;
  error?: string;
}
```

**Client Implementation Details**:
- Finds task by ID in localStorage
- Toggles completed field
- Sets completedAt timestamp if completing
- Clears completedAt if marking incomplete
- Updates updatedAt timestamp
- Saves to localStorage

## 2. User Preferences Contracts

### 2.1 Get User Preferences
**Client Implementation**: Retrieve user preferences from localStorage

```typescript
// Input
interface GetUserPreferencesRequest {
  userId?: string; // For future backend implementation
}

// Output
interface GetUserPreferencesResponse {
  success: true;
  preferences: UserPreferences;
  error?: string;
}
```

### 2.2 Update User Preferences
**Client Implementation**: Update user preferences in localStorage

```typescript
// Input
interface UpdateUserPreferencesRequest {
  userId?: string; // For future backend implementation
  preferences: Partial<UserPreferences>;
}

// Output
interface UpdateUserPreferencesResponse {
  success: true;
  preferences: UserPreferences;
  error?: string;
}
```

## 3. Search & Filter Contracts

### 3.1 Search Tasks
**Client Implementation**: Search tasks using full-text search

```typescript
// Input
interface SearchTasksRequest {
  userId?: string; // For future backend implementation
  query: string;
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
}

// Output
interface SearchTasksResponse {
  success: true;
  tasks: Task[];
  count: number;
  error?: string;
}
```

**Client Implementation Details**:
- Performs case-insensitive search across title, description, and tags
- Applies additional filters if specified
- Sorts results according to specified criteria
- Returns matching tasks with count

## 4. Data Format Specifications

### 4.1 Task Entity Format
```typescript
interface Task {
  id: string;                    // Unique identifier (UUID v4 format)
  title: string;                 // Task title (required, 1-200 chars)
  description?: string;          // Task description (optional, max 1000 chars)
  completed: boolean;            // Completion status (default: false)
  priority: 'low' | 'medium' | 'high';  // Priority level (default: 'medium')
  tags: string[];                // Task tags (max 10 tags, 1-30 chars each)
  dueDate?: string;              // Due date in ISO 8601 format
  recurrence?: {
    type: 'none' | 'daily' | 'weekly' | 'monthly';  // Recurrence interval
    interval: number;           // Interval multiplier (e.g., every 2 weeks)
    endDate?: string;           // End date for recurrence in ISO 8601 format
    occurrences?: number;       // Max number of occurrences (alternative to endDate)
  };
  createdAt: string;             // Creation timestamp (ISO 8601 format)
  updatedAt: string;             // Last update timestamp (ISO 8601 format)
  completedAt?: string;          // Completion timestamp (ISO 8601 format, only when completed=true)
}
```

### 4.2 User Preferences Format
```typescript
interface UserPreferences {
  theme: 'light' | 'dark' | 'system';
  notifications: {
    enabled: boolean;
    dueTimeHours: number;
    deadlineReminders: boolean;
  };
  dashboard: {
    defaultView: 'list' | 'grid';
    showCompleted: boolean;
    defaultSort: 'dueDate' | 'priority' | 'title' | 'created';
    defaultSortOrder: 'asc' | 'desc';
  };
  createdAt: string;
  updatedAt: string;
}
```

## 5. Error Response Format

All API responses follow this error format:

```typescript
interface ErrorResponse {
  success: false;
  error: string;
  errorCode?: string;
  details?: Record<string, any>;  // Specific error details when applicable
}
```

### 5.1 Common Error Codes
- `STORAGE_QUOTA_EXCEEDED`: localStorage quota has been exceeded
- `VALIDATION_ERROR`: Request validation failed
- `TASK_NOT_FOUND`: Task with specified ID does not exist
- `INVALID_TASK_DATA`: Task data does not meet validation requirements
- `STORAGE_UNAVAILABLE`: localStorage is not available in the current environment

## 6. Future Backend Migration Considerations

### 6.1 API Endpoint Mapping (for future implementation)
```typescript
// Current client-side implementation → Future backend endpoint
'/client/tasks' → 'GET /api/tasks'        // Get all tasks
'/client/tasks' → 'POST /api/tasks'       // Create task
'/client/tasks/{id}' → 'PUT /api/tasks/{id}'  // Update task
'/client/tasks/{id}' → 'DELETE /api/tasks/{id}' // Delete task
'/client/preferences' → 'GET /api/preferences'  // Get preferences
'/client/preferences' → 'PUT /api/preferences'  // Update preferences
```

### 6.2 Transition Strategy
1. Abstract data layer with clear interface
2. Implement adapter pattern for easy switching between localStorage and API
3. Maintain identical data formats for smooth transition
4. Gradual rollout of backend features alongside localStorage fallback

## 7. Performance Requirements

### 7.1 Response Time Expectations (Client-side)
- Get tasks: < 50ms for 100 tasks
- Create task: < 10ms
- Update task: < 10ms
- Delete task: < 10ms
- Search tasks: < 100ms for 100 tasks with complex filters

### 7.2 Storage Limits
- Maximum tasks per user: 10,000 (with 5-10MB localStorage allocation)
- Individual task size limit: 1KB (including all fields)
- Total storage allocation: 5-10MB for task data