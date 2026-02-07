# API Contracts: TASKAPP Client-Side Operations

**Feature**: 1-ui-redesign-routing
**Created**: 2026-01-16

## Overview

These contracts define the client-side data operations for TASKAPP's local workspace functionality. Since the application is transitioning to public access without authentication, all operations will be handled client-side using local storage and React state management.

## Core Operations

### Task Management Contracts

#### GET /local/tasks
**Description**: Retrieve all tasks in the local workspace

**Request**:
- Method: GET
- Parameters: None
- Authentication: Not required

**Response**:
```json
{
  "success": true,
  "data": [
    {
      "id": "string",
      "title": "string",
      "description": "string | null",
      "completed": "boolean",
      "priority": "string | null",
      "tags": "string[]",
      "dueDate": "string | null",
      "recurrence": "string | null",
      "createdAt": "string",
      "updatedAt": "string"
    }
  ],
  "message": "string | null"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "type": "StorageError | ValidationError",
    "message": "string"
  }
}
```

**Success Criteria**:
- Returns all tasks in the local workspace
- Maintains proper ordering and structure
- Handles empty workspace gracefully

---

#### POST /local/tasks
**Description**: Create a new task in the local workspace

**Request**:
```json
{
  "title": "string (1-100 chars)",
  "description": "string (0-5000 chars, optional)",
  "priority": "string (high|medium|low, optional)",
  "tags": "string[] (each 1-30 chars, optional)",
  "dueDate": "string (ISO 8601, optional)",
  "recurrence": "string (none|daily|weekly, optional)"
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "string",
    "title": "string",
    "description": "string | null",
    "completed": "boolean",
    "priority": "string | null",
    "tags": "string[]",
    "dueDate": "string | null",
    "recurrence": "string | null",
    "createdAt": "string",
    "updatedAt": "string"
  },
  "message": "string | null"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "type": "ValidationError | StorageError",
    "message": "string",
    "details": [
      {
        "field": "string",
        "issue": "string"
      }
    ]
  }
}
```

**Success Criteria**:
- Creates new task with all provided properties
- Sets default values for optional fields
- Generates unique ID and timestamps
- Updates workspace and persists to storage

---

#### GET /local/tasks/{taskId}
**Description**: Retrieve a specific task by ID

**Request**:
- Method: GET
- Path Parameter: taskId (string)
- Authentication: Not required

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "string",
    "title": "string",
    "description": "string | null",
    "completed": "boolean",
    "priority": "string | null",
    "tags": "string[]",
    "dueDate": "string | null",
    "recurrence": "string | null",
    "createdAt": "string",
    "updatedAt": "string"
  },
  "message": "string | null"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "type": "NotFoundError | StorageError",
    "message": "string"
  }
}
```

**Success Criteria**:
- Returns the specified task if it exists
- Returns appropriate error if task not found
- Maintains data integrity

---

#### PUT /local/tasks/{taskId}
**Description**: Update an existing task in the local workspace

**Request**:
```json
{
  "title": "string (1-100 chars, optional)",
  "description": "string (0-5000 chars, optional)",
  "completed": "boolean (optional)",
  "priority": "string (high|medium|low, optional)",
  "tags": "string[] (each 1-30 chars, optional)",
  "dueDate": "string (ISO 8601, optional)",
  "recurrence": "string (none|daily|weekly, optional)"
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "string",
    "title": "string",
    "description": "string | null",
    "completed": "boolean",
    "priority": "string | null",
    "tags": "string[]",
    "dueDate": "string | null",
    "recurrence": "string | null",
    "createdAt": "string",
    "updatedAt": "string"
  },
  "message": "string | null"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "type": "NotFoundError | ValidationError | StorageError",
    "message": "string"
  }
}
```

**Success Criteria**:
- Updates only the specified fields (partial updates)
- Preserves unchanged fields
- Updates the updatedAt timestamp
- Validates all provided data

---

#### DELETE /local/tasks/{taskId}
**Description**: Delete a specific task from the local workspace

**Request**:
- Method: DELETE
- Path Parameter: taskId (string)
- Authentication: Not required

**Response**:
```json
{
  "success": true,
  "data": null,
  "message": "string | null"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "type": "NotFoundError | StorageError",
    "message": "string"
  }
}
```

**Success Criteria**:
- Removes the specified task from the workspace
- Returns success if task existed and was removed
- Returns appropriate error if task not found

---

#### PATCH /local/tasks/{taskId}/complete
**Description**: Toggle the completion status of a task

**Request**:
```json
{
  "completed": "boolean (optional)"
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "string",
    "title": "string",
    "description": "string | null",
    "completed": "boolean",
    "priority": "string | null",
    "tags": "string[]",
    "dueDate": "string | null",
    "recurrence": "string | null",
    "createdAt": "string",
    "updatedAt": "string"
  },
  "message": "string | null"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "type": "NotFoundError | StorageError",
    "message": "string"
  }
}
```

**Success Criteria**:
- Toggles the completed status if no value provided
- Sets completed status to specified value if provided
- Updates the updatedAt timestamp

---

## Workspace Management Contracts

#### GET /local/workspace
**Description**: Retrieve the entire local workspace state

**Request**:
- Method: GET
- Authentication: Not required

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "string",
    "tasks": "[Array of Task objects]",
    "lastModified": "string",
    "settings": {
      "theme": "light | dark",
      "notificationsEnabled": "boolean",
      "defaultPriority": "high | medium | low"
    }
  },
  "message": "string | null"
}
```

**Success Criteria**:
- Returns complete workspace state
- Includes all tasks and settings
- Maintains data structure integrity

---

#### PUT /local/workspace/settings
**Description**: Update workspace settings

**Request**:
```json
{
  "theme": "light | dark (optional)",
  "notificationsEnabled": "boolean (optional)",
  "defaultPriority": "high | medium | low (optional)"
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "theme": "light | dark",
    "notificationsEnabled": "boolean",
    "defaultPriority": "high | medium | low"
  },
  "message": "string | null"
}
```

**Success Criteria**:
- Updates only the specified settings
- Preserves unchanged settings
- Returns updated values

---

## Error Types

### Standard Error Responses

**ValidationError**:
- **Status**: Client-side validation failure
- **Message**: Describes the validation issue
- **Details**: Array of specific field validation errors

**NotFoundError**:
- **Status**: Requested resource not found
- **Message**: Specifies which resource was not found

**StorageError**:
- **Status**: Local storage or persistence failure
- **Message**: Describes the storage issue
- **Cause**: May indicate storage quota exceeded, etc.

### HTTP Status Code Mapping (Client-Side)

- **200 OK**: Operation completed successfully
- **400 Bad Request**: Validation error occurred
- **404 Not Found**: Requested resource does not exist
- **500 Internal Error**: Storage or system error

## Implementation Notes

1. **Client-Side Implementation**: These contracts represent client-side operations that interact with local storage and React state management rather than traditional HTTP APIs.

2. **Response Format**: All operations return the same standardized response format for consistency.

3. **Data Validation**: Client-side validation must match the validation rules defined in the data model.

4. **Persistence**: All operations should persist changes to localStorage or similar client-side storage mechanism.

5. **Error Boundaries**: Operations should implement proper error handling to prevent UI crashes.