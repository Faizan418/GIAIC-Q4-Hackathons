# API Specification: Todo Application

## Overview
This document defines the API contracts for the Todo application, including authentication, user management, and task management endpoints. The API follows RESTful principles with JWT-based authentication.

## Base URL
```
https://your-domain.com/api
```
For local development: `http://localhost:8000/api`

## Authentication
All protected endpoints require a valid JWT token in the Authorization header:
```
Authorization: Bearer {jwt_token}
```

## Common Response Format
The API uses a consistent response wrapper format:

### Success Response
```json
{
  "success": true,
  "data": { ... },
  "message": "Optional success message"
}
```

### Error Response
```json
{
  "success": false,
  "error": {
    "type": "ErrorType",
    "message": "Error description"
  }
}
```

## Endpoints

### Authentication Endpoints

#### POST /api/auth/register
**Description**: Register a new user account

**Request Body**:
```json
{
  "name": "string (required, 1-50 characters)",
  "email": "string (required, valid email format)",
  "password": "string (required, min 8 characters)"
}
```

**Response 200**:
```json
{
  "id": "string (UUID)",
  "email": "string",
  "name": "string"
}
```

**Error Responses**:
- 400 Bad Request → `{ "success": false, "error": { "type": "HTTPException", "message": "User with this email already exists" } }`
- 422 Validation Error → `{ "success": false, "error": { "type": "ValidationError", "details": [...] } }`

#### POST /api/auth/login
**Description**: Authenticate user and obtain JWT token

**Request Body**:
```json
{
  "email": "string (required)",
  "password": "string (required)"
}
```

**Response 200**:
```json
{
  "access_token": "string (JWT token)",
  "token_type": "bearer"
}
```

**Error Responses**:
- 401 Unauthorized → `{ "success": false, "error": { "type": "HTTPException", "message": "Incorrect email or password" } }`
- 422 Validation Error → `{ "success": false, "error": { "type": "ValidationError", "details": [...] } }`

#### GET /api/auth/me
**Description**: Get current authenticated user's profile

**Authentication**: Required (JWT Bearer)

**Response 200**:
```json
{
  "id": "string (UUID)",
  "email": "string",
  "name": "string"
}
```

**Error Responses**:
- 401 Unauthorized → Invalid or expired token
- 403 Forbidden → Insufficient permissions

### Task Management Endpoints

#### GET /api/tasks/
**Description**: Retrieve all tasks for the authenticated user with optional filtering

**Authentication**: Required (JWT Bearer)

**Query Parameters**:
- `status` (optional): Filter by completion status ("completed", "pending", "all")
- `limit` (optional): Number of tasks to return (default: 100, max: 1000)
- `offset` (optional): Number of tasks to skip for pagination (default: 0)

**Response 200**:
```json
[
  {
    "id": "string (UUID)",
    "user_id": "string (UUID)",
    "title": "string",
    "description": "string or null",
    "completed": "boolean",
    "priority": "string ('high', 'medium', 'low') or null",
    "tags": "array of strings",
    "due_date": "ISO 8601 datetime string or null",
    "recurrence": "string ('none', 'daily', 'weekly')",
    "created_at": "ISO 8601 datetime string",
    "updated_at": "ISO 8601 datetime string"
  }
]
```

**Error Responses**:
- 401 Unauthorized → Invalid or expired token
- 403 Forbidden → User not authorized to access these tasks
- 422 Validation Error → Invalid query parameters

#### POST /api/tasks/
**Description**: Create a new task for the authenticated user

**Authentication**: Required (JWT Bearer)

**Request Body**:
```json
{
  "title": "string (required, 1-100 characters)",
  "description": "string (optional, max 5000 characters)",
  "priority": "string (optional, 'high', 'medium', 'low')",
  "tags": "array of strings (optional)",
  "due_date": "ISO 8601 datetime string (optional)",
  "recurrence": "string (optional, 'none', 'daily', 'weekly')"
}
```

**Response 200**:
```json
{
  "id": "string (UUID)",
  "user_id": "string (UUID)",
  "title": "string",
  "description": "string or null",
  "completed": "boolean (default: false)",
  "priority": "string or null",
  "tags": "array of strings",
  "due_date": "ISO 8601 datetime string or null",
  "recurrence": "string",
  "created_at": "ISO 8601 datetime string",
  "updated_at": "ISO 8601 datetime string"
}
```

**Error Responses**:
- 401 Unauthorized → Invalid or expired token
- 403 Forbidden → User not authorized to create tasks
- 422 Validation Error → Invalid request body

#### GET /api/tasks/{task_id}
**Description**: Retrieve a specific task by ID

**Path Parameters**:
- `task_id`: string (required) - UUID of the task to retrieve

**Authentication**: Required (JWT Bearer)

**Response 200**:
```json
{
  "id": "string (UUID)",
  "user_id": "string (UUID)",
  "title": "string",
  "description": "string or null",
  "completed": "boolean",
  "priority": "string or null",
  "tags": "array of strings",
  "due_date": "ISO 8601 datetime string or null",
  "recurrence": "string",
  "created_at": "ISO 8601 datetime string",
  "updated_at": "ISO 8601 datetime string"
}
```

**Error Responses**:
- 401 Unauthorized → Invalid or expired token
- 403 Forbidden → User not authorized to access this task
- 404 Not Found → `{ "success": false, "error": { "type": "HTTPException", "message": "Task not found" } }`
- 422 Validation Error → Invalid task_id format

#### PUT /api/tasks/{task_id}
**Description**: Update a specific task by ID

**Path Parameters**:
- `task_id`: string (required) - UUID of the task to update

**Authentication**: Required (JWT Bearer)

**Request Body** (all fields optional for partial updates):
```json
{
  "title": "string (optional)",
  "description": "string (optional)",
  "completed": "boolean (optional)",
  "priority": "string (optional)",
  "tags": "array of strings (optional)",
  "due_date": "ISO 8601 datetime string (optional)",
  "recurrence": "string (optional)"
}
```

**Response 200**:
```json
{
  "id": "string (UUID)",
  "user_id": "string (UUID)",
  "title": "string",
  "description": "string or null",
  "completed": "boolean",
  "priority": "string or null",
  "tags": "array of strings",
  "due_date": "ISO 8601 datetime string or null",
  "recurrence": "string",
  "created_at": "ISO 8601 datetime string",
  "updated_at": "ISO 8601 datetime string"
}
```

**Error Responses**:
- 401 Unauthorized → Invalid or expired token
- 403 Forbidden → User not authorized to update this task
- 404 Not Found → `{ "success": false, "error": { "type": "HTTPException", "message": "Task not found" } }`
- 422 Validation Error → Invalid request body or task_id format

#### PATCH /api/tasks/{task_id}/complete
**Description**: Toggle the completion status of a specific task

**Path Parameters**:
- `task_id`: string (required) - UUID of the task to update

**Authentication**: Required (JWT Bearer)

**Request Body**:
```json
{
  "completed": "boolean (optional) - If provided, sets the completion status directly"
}
```

**Response 200**:
```json
{
  "id": "string (UUID)",
  "user_id": "string (UUID)",
  "title": "string",
  "description": "string or null",
  "completed": "boolean",
  "priority": "string or null",
  "tags": "array of strings",
  "due_date": "ISO 8601 datetime string or null",
  "recurrence": "string",
  "created_at": "ISO 8601 datetime string",
  "updated_at": "ISO 8601 datetime string"
}
```

**Error Responses**:
- 401 Unauthorized → Invalid or expired token
- 403 Forbidden → User not authorized to update this task
- 404 Not Found → `{ "success": false, "error": { "type": "HTTPException", "message": "Task not found" } }`
- 422 Validation Error → Invalid task_id format or request body

#### DELETE /api/tasks/{task_id}
**Description**: Delete a specific task by ID

**Path Parameters**:
- `task_id`: string (required) - UUID of the task to delete

**Authentication**: Required (JWT Bearer)

**Response 200**:
```json
{
  "message": "Task deleted successfully"
}
```

**Error Responses**:
- 401 Unauthorized → Invalid or expired token
- 403 Forbidden → User not authorized to delete this task
- 404 Not Found → `{ "success": false, "error": { "type": "HTTPException", "message": "Task not found" } }`
- 422 Validation Error → Invalid task_id format

### Health Check Endpoint

#### GET /
**Description**: Root endpoint - returns welcome message

**Response 200**:
```json
{
  "message": "Todo API - Welcome!"
}
```

#### GET /health
**Description**: Health check endpoint - verifies API availability

**Response 200**:
```json
{
  "status": "healthy"
}
```

## Error Handling

### HTTP Status Codes
- `200 OK`: Successful request
- `201 Created`: Resource successfully created
- `400 Bad Request`: Invalid request format
- `401 Unauthorized`: Missing or invalid authentication
- `403 Forbidden`: Valid authentication but insufficient permissions
- `404 Not Found`: Requested resource does not exist
- `422 Unprocessable Entity`: Request validation failed
- `500 Internal Server Error`: Server error occurred

### Common Error Types
- `HTTPException`: General HTTP errors with specific messages
- `ValidationError`: Request body or parameter validation failures
- `AuthenticationError`: JWT token validation failures
- `AuthorizationError`: Permission-related errors
- `NotFoundError`: Resource not found errors

## Security Considerations
- All endpoints except health check require JWT authentication
- User data isolation ensures users can only access their own tasks
- Input validation prevents injection attacks
- Rate limiting (100 requests/IP/hour) prevents abuse
- Passwords are hashed using bcrypt before storage
- All datetime values are stored in UTC

## Versioning
This API follows semantic versioning. Breaking changes will increment the major version number. Current version: 1.0.0

## Rate Limits
- 100 requests per IP address per hour
- Exceeded requests return 429 Too Many Requests status code

## Pagination
Most list endpoints support pagination using `limit` and `offset` query parameters:
- `limit`: Number of items to return (default: 100, max: 1000)
- `offset`: Number of items to skip for pagination (default: 0)