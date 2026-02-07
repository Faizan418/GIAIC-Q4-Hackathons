# API Contracts: Frontend Todo App

## Authentication Endpoints

### Register User
- **Endpoint**: `POST /api/auth/register`
- **Request**:
  ```json
  {
    "email": "user@example.com",
    "password": "securePassword123",
    "name": "John Doe"
  }
  ```
- **Response (Success)**:
  ```json
  {
    "user": {
      "id": "uuid-string",
      "email": "user@example.com",
      "name": "John Doe",
      "createdAt": "2026-01-11T10:00:00.000Z"
    },
    "token": "jwt-token-string"
  }
  ```
- **Response (Error)**:
  ```json
  {
    "error": "Email already exists",
    "code": "EMAIL_EXISTS"
  }
  ```
- **Authentication**: None required

### Login User
- **Endpoint**: `POST /api/auth/login`
- **Request**:
  ```json
  {
    "email": "user@example.com",
    "password": "securePassword123"
  }
  ```
- **Response (Success)**:
  ```json
  {
    "user": {
      "id": "uuid-string",
      "email": "user@example.com",
      "name": "John Doe"
    },
    "token": "jwt-token-string"
  }
  ```
- **Response (Error)**:
  ```json
  {
    "error": "Invalid credentials",
    "code": "INVALID_CREDENTIALS"
  }
  ```
- **Authentication**: None required

### Logout User
- **Endpoint**: `POST /api/auth/logout`
- **Request**: Empty body
- **Response (Success)**:
  ```json
  {
    "message": "Logged out successfully"
  }
  ```
- **Authentication**: JWT Token required in Authorization header

## Task Management Endpoints

### Get All User Tasks
- **Endpoint**: `GET /api/users/{user_id}/tasks`
- **Query Parameters**:
  - `search`: Keyword to search in title, description, tags, priority, status, and timestamps
  - `status`: Filter by status ('pending', 'completed', 'all')
  - `priority`: Filter by priority ('high', 'medium', 'low')
  - `startDate`: Filter by due date range start (ISO date string)
  - `endDate`: Filter by due date range end (ISO date string)
  - `sortBy`: Sort by ('dueDate', 'priority', 'title', 'createdAt')
  - `sortOrder`: Sort order ('asc', 'desc')
- **Response (Success)**:
  ```json
  {
    "tasks": [
      {
        "id": "task-uuid",
        "title": "Sample Task",
        "description": "Task description",
        "status": "pending",
        "priority": "high",
        "tags": ["work", "important"],
        "dueDate": "2026-01-15T10:00:00.000Z",
        "recurrencePattern": "none",
        "userId": "user-uuid",
        "createdAt": "2026-01-11T10:00:00.000Z",
        "updatedAt": "2026-01-11T10:00:00.000Z"
      }
    ],
    "pagination": {
      "page": 1,
      "limit": 20,
      "total": 1,
      "hasNext": false
    }
  }
  ```
- **Authentication**: JWT Token required in Authorization header

### Create Task
- **Endpoint**: `POST /api/users/{user_id}/tasks`
- **Request**:
  ```json
  {
    "title": "New Task",
    "description": "Task description",
    "status": "pending",
    "priority": "medium",
    "tags": ["personal"],
    "dueDate": "2026-01-15T10:00:00.000Z",
    "recurrencePattern": "none"
  }
  ```
- **Response (Success)**:
  ```json
  {
    "task": {
      "id": "task-uuid",
      "title": "New Task",
      "description": "Task description",
      "status": "pending",
      "priority": "medium",
      "tags": ["personal"],
      "dueDate": "2026-01-15T10:00:00.000Z",
      "recurrencePattern": "none",
      "userId": "user-uuid",
      "createdAt": "2026-01-11T10:00:00.000Z",
      "updatedAt": "2026-01-11T10:00:00.000Z"
    }
  }
  ```
- **Response (Error)**:
  ```json
  {
    "error": "Title is required",
    "code": "VALIDATION_ERROR"
  }
  ```
- **Authentication**: JWT Token required in Authorization header

### Update Task
- **Endpoint**: `PUT /api/users/{user_id}/tasks/{task_id}`
- **Request**:
  ```json
  {
    "title": "Updated Task Title",
    "description": "Updated description",
    "status": "completed",
    "priority": "high",
    "tags": ["work", "urgent"],
    "dueDate": "2026-01-20T10:00:00.000Z",
    "recurrencePattern": "daily"
  }
  ```
- **Response (Success)**:
  ```json
  {
    "task": {
      "id": "task-uuid",
      "title": "Updated Task Title",
      "description": "Updated description",
      "status": "completed",
      "priority": "high",
      "tags": ["work", "urgent"],
      "dueDate": "2026-01-20T10:00:00.000Z",
      "recurrencePattern": "daily",
      "userId": "user-uuid",
      "createdAt": "2026-01-11T10:00:00.000Z",
      "updatedAt": "2026-01-11T11:00:00.000Z"
    }
  }
  ```
- **Authentication**: JWT Token required in Authorization header

### Delete Task
- **Endpoint**: `DELETE /api/users/{user_id}/tasks/{task_id}`
- **Response (Success)**:
  ```json
  {
    "message": "Task deleted successfully"
  }
  ```
- **Authentication**: JWT Token required in Authorization header

### Toggle Task Completion
- **Endpoint**: `PATCH /api/users/{user_id}/tasks/{task_id}/complete`
- **Request**:
  ```json
  {
    "status": "completed" // or "pending"
  }
  ```
- **Response (Success)**:
  ```json
  {
    "task": {
      "id": "task-uuid",
      "title": "Sample Task",
      "status": "completed",
      "updatedAt": "2026-01-11T11:00:00.000Z"
    }
  }
  ```
- **Authentication**: JWT Token required in Authorization header

## Error Response Format

All error responses follow this format:
```json
{
  "error": "Human-readable error message",
  "code": "ERROR_CODE_STRING",
  "timestamp": "2026-01-11T10:00:00.000Z",
  "path": "/api/users/user-id/tasks"
}
```

## Common Error Codes

- `UNAUTHORIZED`: User is not authenticated or token is invalid
- `FORBIDDEN`: User does not have permission for this action
- `NOT_FOUND`: Requested resource does not exist
- `VALIDATION_ERROR`: Request data failed validation
- `INTERNAL_ERROR`: Unexpected server error occurred

## Headers

### Request Headers
- `Authorization: Bearer {jwt_token}` - Required for authenticated endpoints
- `Content-Type: application/json` - For JSON payloads

### Response Headers
- `Content-Type: application/json` - For JSON responses
- `Cache-Control: no-cache` - For authenticated endpoints