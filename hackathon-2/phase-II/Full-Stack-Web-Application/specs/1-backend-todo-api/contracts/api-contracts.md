# API Contracts: Backend Todo API

## Base URL
`/api/{user_id}/`

All endpoints require JWT authentication and enforce user_id matching between token and URL.

## Authentication
All endpoints require a valid JWT token in the Authorization header:
```
Authorization: Bearer {jwt_token}
```

## Common Response Format

### Success Responses
```json
{
  "success": true,
  "data": { /* response data */ },
  "message": "Optional message"
}
```

### Error Responses
```json
{
  "success": false,
  "error": {
    "code": "ERROR_CODE",
    "message": "Error message",
    "details": { /* optional error details */ }
  }
}
```

## Endpoints

### GET /tasks
Retrieve all tasks for the authenticated user with optional filtering and sorting.

**Query Parameters:**
- `status`: 'all'/'pending'/'completed' (default: 'all')
- `search`: keyword to search in title/description
- `priority`: 'high'/'medium'/'low' (optional filter)
- `date_from`: ISO datetime for date range filtering
- `date_to`: ISO datetime for date range filtering
- `sort`: 'due_date'/'priority'/'title'/'created_at' (default: 'created_at')
- `order`: 'asc'/'desc' (default: 'desc')

**Response:**
```json
{
  "success": true,
  "data": {
    "tasks": [
      {
        "id": 1,
        "user_id": "user-123",
        "title": "Sample task",
        "description": "Sample description",
        "completed": false,
        "priority": "medium",
        "tags": ["tag1", "tag2"],
        "due_date": "2023-12-31T10:00:00Z",
        "recurrence": "none",
        "created_at": "2023-01-01T10:00:00Z",
        "updated_at": "2023-01-01T10:00:00Z"
      }
    ],
    "total": 1,
    "page": 1,
    "limit": 50
  }
}
```

**HTTP Status Codes:**
- 200: Success
- 401: Unauthorized
- 403: Forbidden (user_id mismatch)
- 422: Validation error

### POST /tasks
Create a new task for the authenticated user.

**Request Body:**
```json
{
  "title": "New task",
  "description": "Task description",
  "priority": "medium",
  "tags": ["work", "important"],
  "due_date": "2023-12-31T10:00:00Z",
  "recurrence": "none"
}
```

**Validation:**
- `title`: Required, 1-100 characters
- `description`: Optional, max 5000 characters
- `priority`: Optional, one of 'high', 'medium', 'low'
- `tags`: Optional, array of strings
- `due_date`: Optional, ISO datetime string
- `recurrence`: Optional, one of 'none', 'daily', 'weekly'

**Response:**
```json
{
  "success": true,
  "data": {
    "id": 1,
    "user_id": "user-123",
    "title": "New task",
    "description": "Task description",
    "completed": false,
    "priority": "medium",
    "tags": ["work", "important"],
    "due_date": "2023-12-31T10:00:00Z",
    "recurrence": "none",
    "created_at": "2023-01-01T10:00:00Z",
    "updated_at": "2023-01-01T10:00:00Z"
  }
}
```

**HTTP Status Codes:**
- 201: Created
- 400: Bad request (validation error)
- 401: Unauthorized
- 403: Forbidden (user_id mismatch)

### GET /tasks/{id}
Retrieve a specific task by ID.

**Response:**
```json
{
  "success": true,
  "data": {
    "id": 1,
    "user_id": "user-123",
    "title": "Sample task",
    "description": "Sample description",
    "completed": false,
    "priority": "medium",
    "tags": ["tag1", "tag2"],
    "due_date": "2023-12-31T10:00:00Z",
    "recurrence": "none",
    "created_at": "2023-01-01T10:00:00Z",
    "updated_at": "2023-01-01T10:00:00Z"
  }
}
```

**HTTP Status Codes:**
- 200: Success
- 401: Unauthorized
- 403: Forbidden (user_id mismatch)
- 404: Task not found (either doesn't exist or belongs to another user)

### PUT /tasks/{id}
Update an existing task.

**Request Body (partial updates allowed):**
```json
{
  "title": "Updated task title",
  "description": "Updated description",
  "priority": "high",
  "tags": ["work", "urgent"],
  "due_date": "2023-12-31T10:00:00Z",
  "recurrence": "daily",
  "completed": false
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "id": 1,
    "user_id": "user-123",
    "title": "Updated task title",
    "description": "Updated description",
    "completed": false,
    "priority": "high",
    "tags": ["work", "urgent"],
    "due_date": "2023-12-31T10:00:00Z",
    "recurrence": "daily",
    "created_at": "2023-01-01T10:00:00Z",
    "updated_at": "2023-01-01T11:00:00Z"
  }
}
```

**HTTP Status Codes:**
- 200: Success
- 400: Bad request (validation error)
- 401: Unauthorized
- 403: Forbidden (user_id mismatch)
- 404: Task not found

### DELETE /tasks/{id}
Delete a task.

**Response:**
```json
{
  "success": true,
  "message": "Task deleted successfully"
}
```

**HTTP Status Codes:**
- 200: Deleted successfully
- 401: Unauthorized
- 403: Forbidden (user_id mismatch)
- 404: Task not found

### PATCH /tasks/{id}/complete
Toggle the completion status of a task.

**Request Body (optional):**
```json
{
  "completed": true
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "id": 1,
    "user_id": "user-123",
    "title": "Sample task",
    "description": "Sample description",
    "completed": true,
    "priority": "medium",
    "tags": ["tag1", "tag2"],
    "due_date": "2023-12-31T10:00:00Z",
    "recurrence": "daily",  // If this was a recurring task
    "created_at": "2023-01-01T10:00:00Z",
    "updated_at": "2023-01-01T11:00:00Z"
  }
}
```

If the task was recurring and completed, a new task instance may be created and returned in an additional field:

```json
{
  "success": true,
  "data": {
    // ... updated original task
  },
  "next_occurrence": {
    // ... new task instance for next occurrence
  }
}
```

**HTTP Status Codes:**
- 200: Success
- 400: Bad request (invalid completed value)
- 401: Unauthorized
- 403: Forbidden (user_id mismatch)
- 404: Task not found

## Error Codes

| Code | Description |
|------|-------------|
| AUTH_001 | Invalid or expired token |
| AUTH_002 | User ID mismatch |
| VALIDATION_001 | Validation error |
| RESOURCE_001 | Resource not found |
| SERVER_001 | Internal server error |

## Rate Limiting
All endpoints are subject to rate limiting: maximum 100 requests per IP per hour.