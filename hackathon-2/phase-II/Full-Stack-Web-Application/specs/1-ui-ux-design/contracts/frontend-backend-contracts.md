# API Contracts: TASKAPP Frontend-Backend Integration

## 1. Authentication Endpoints

### 1.1 Sign Up
```
POST /api/auth/signup
```

**Request**:
```json
{
  "email": "user@example.com",
  "password": "securePassword123",
  "name": "John Doe"
}
```

**Response**:
```json
{
  "success": true,
  "user": {
    "id": "user_12345",
    "email": "user@example.com",
    "name": "John Doe",
    "createdAt": "2026-01-14T10:30:00Z"
  },
  "token": "jwt_token_here"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": "Invalid email format"
}
```

**Validation**:
- Email must be valid email format
- Password must be at least 8 characters
- Name is optional, max 100 characters

### 1.2 Sign In
```
POST /api/auth/signin
```

**Request**:
```json
{
  "email": "user@example.com",
  "password": "securePassword123"
}
```

**Response**:
```json
{
  "success": true,
  "user": {
    "id": "user_12345",
    "email": "user@example.com",
    "name": "John Doe",
    "createdAt": "2026-01-14T10:30:00Z"
  },
  "token": "jwt_token_here"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": "Invalid credentials"
}
```

### 1.3 Google OAuth
```
GET /api/auth/google
```

**Response**:
- Redirects to Google OAuth flow
- On successful completion, redirects back with JWT token

### 1.4 Sign Out
```
POST /api/auth/signout
```

**Headers**:
```
Authorization: Bearer {jwt_token}
```

**Response**:
```json
{
  "success": true
}
```

## 2. Task Management Endpoints

### 2.1 Get User Tasks
```
GET /api/users/{userId}/tasks
```

**Headers**:
```
Authorization: Bearer {jwt_token}
```

**Query Parameters**:
- `completed` (optional): boolean - filter by completion status
- `priority` (optional): string - filter by priority ('low', 'medium', 'high')
- `due_date_from` (optional): string - filter tasks due after date
- `due_date_to` (optional): string - filter tasks due before date
- `tags` (optional): string - filter by tags (comma-separated)
- `sort_by` (optional): string - sort by 'created_at', 'due_date', 'priority' (default: 'created_at')
- `sort_order` (optional): string - 'asc' or 'desc' (default: 'desc')
- `page` (optional): number - page number (default: 1)
- `page_size` (optional): number - items per page (default: 20, max: 100)

**Response**:
```json
{
  "success": true,
  "tasks": [
    {
      "id": "task_12345",
      "userId": "user_12345",
      "title": "Complete project proposal",
      "description": "Finish the proposal document for client review",
      "completed": false,
      "priority": "high",
      "tags": ["work", "urgent"],
      "dueDate": "2026-01-20T09:00:00Z",
      "recurrence": "none",
      "createdAt": "2026-01-14T10:30:00Z",
      "updatedAt": "2026-01-14T10:30:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "total": 1,
    "hasNext": false,
    "hasPrev": false
  }
}
```

### 2.2 Create Task
```
POST /api/users/{userId}/tasks
```

**Headers**:
```
Authorization: Bearer {jwt_token}
Content-Type: application/json
```

**Request**:
```json
{
  "title": "Complete project proposal",
  "description": "Finish the proposal document for client review",
  "priority": "high",
  "tags": ["work", "urgent"],
  "dueDate": "2026-01-20T09:00:00Z",
  "recurrence": "none"
}
```

**Response**:
```json
{
  "success": true,
  "task": {
    "id": "task_12345",
    "userId": "user_12345",
    "title": "Complete project proposal",
    "description": "Finish the proposal document for client review",
    "completed": false,
    "priority": "high",
    "tags": ["work", "urgent"],
    "dueDate": "2026-01-20T09:00:00Z",
    "recurrence": "none",
    "createdAt": "2026-01-14T10:30:00Z",
    "updatedAt": "2026-01-14T10:30:00Z"
  }
}
```

**Validation**:
- Title is required, 1-200 characters
- Description is optional, max 1000 characters
- Priority is required, must be 'low', 'medium', or 'high'
- Tags is optional, max 10 tags, each 1-30 characters
- Due date is optional, must be valid ISO date format if provided
- Recurrence is required, must be 'none', 'daily', 'weekly', or 'monthly'

### 2.3 Get Task
```
GET /api/users/{userId}/tasks/{taskId}
```

**Headers**:
```
Authorization: Bearer {jwt_token}
```

**Response**:
```json
{
  "success": true,
  "task": {
    "id": "task_12345",
    "userId": "user_12345",
    "title": "Complete project proposal",
    "description": "Finish the proposal document for client review",
    "completed": false,
    "priority": "high",
    "tags": ["work", "urgent"],
    "dueDate": "2026-01-20T09:00:00Z",
    "recurrence": "none",
    "createdAt": "2026-01-14T10:30:00Z",
    "updatedAt": "2026-01-14T10:30:00Z"
  }
}
```

### 2.4 Update Task
```
PUT /api/users/{userId}/tasks/{taskId}
```

**Headers**:
```
Authorization: Bearer {jwt_token}
Content-Type: application/json
```

**Request**:
```json
{
  "title": "Updated project proposal",
  "description": "Revised proposal document for client review",
  "priority": "medium",
  "tags": ["work", "important"],
  "dueDate": "2026-01-22T09:00:00Z",
  "recurrence": "none",
  "completed": true
}
```

**Response**:
```json
{
  "success": true,
  "task": {
    "id": "task_12345",
    "userId": "user_12345",
    "title": "Updated project proposal",
    "description": "Revised proposal document for client review",
    "completed": true,
    "priority": "medium",
    "tags": ["work", "important"],
    "dueDate": "2026-01-22T09:00:00Z",
    "recurrence": "none",
    "createdAt": "2026-01-14T10:30:00Z",
    "updatedAt": "2026-01-14T11:00:00Z"
  }
}
```

### 2.5 Update Task Completion
```
PATCH /api/users/{userId}/tasks/{taskId}/completion
```

**Headers**:
```
Authorization: Bearer {jwt_token}
Content-Type: application/json
```

**Request**:
```json
{
  "completed": true
}
```

**Response**:
```json
{
  "success": true,
  "task": {
    "id": "task_12345",
    "userId": "user_12345",
    "title": "Complete project proposal",
    "description": "Finish the proposal document for client review",
    "completed": true,
    "priority": "high",
    "tags": ["work", "urgent"],
    "dueDate": "2026-01-20T09:00:00Z",
    "recurrence": "none",
    "createdAt": "2026-01-14T10:30:00Z",
    "updatedAt": "2026-01-14T11:00:00Z"
  }
}
```

### 2.6 Delete Task
```
DELETE /api/users/{userId}/tasks/{taskId}
```

**Headers**:
```
Authorization: Bearer {jwt_token}
```

**Response**:
```json
{
  "success": true
}
```

## 3. User Preferences Endpoints

### 3.1 Get User Preferences
```
GET /api/users/{userId}/preferences
```

**Headers**:
```
Authorization: Bearer {jwt_token}
```

**Response**:
```json
{
  "success": true,
  "preferences": {
    "theme": {
      "userId": "user_12345",
      "theme": "dark",
      "updatedAt": "2026-01-14T10:30:00Z"
    },
    "notifications": {
      "userId": "user_12345",
      "emailNotifications": true,
      "emailFrequency": "immediate",
      "deadlineReminders": true,
      "deadlineHoursBefore": 24,
      "updatedAt": "2026-01-14T10:30:00Z"
    }
  }
}
```

### 3.2 Update Theme Preference
```
PUT /api/users/{userId}/preferences/theme
```

**Headers**:
```
Authorization: Bearer {jwt_token}
Content-Type: application/json
```

**Request**:
```json
{
  "theme": "light"
}
```

**Response**:
```json
{
  "success": true,
  "theme": {
    "userId": "user_12345",
    "theme": "light",
    "updatedAt": "2026-01-14T10:30:00Z"
  }
}
```

### 3.3 Update Notification Preferences
```
PUT /api/users/{userId}/preferences/notifications
```

**Headers**:
```
Authorization: Bearer {jwt_token}
Content-Type: application/json
```

**Request**:
```json
{
  "emailNotifications": true,
  "emailFrequency": "daily",
  "deadlineReminders": true,
  "deadlineHoursBefore": 48
}
```

**Response**:
```json
{
  "success": true,
  "notifications": {
    "userId": "user_12345",
    "emailNotifications": true,
    "emailFrequency": "daily",
    "deadlineReminders": true,
    "deadlineHoursBefore": 48,
    "updatedAt": "2026-01-14T10:30:00Z"
  }
}
```

## 4. Error Response Format

All endpoints return errors in the following format:

```json
{
  "success": false,
  "error": "Human-readable error message",
  "errorCode": "ERROR_CODE_CONSTANT",
  "timestamp": "2026-01-14T10:30:00Z"
}
```

## 5. Common Headers

All authenticated endpoints require:
```
Authorization: Bearer {jwt_token}
```

All POST/PUT/PATCH requests with JSON bodies require:
```
Content-Type: application/json
```

## 6. Common Query Parameters

- `page`: Page number for pagination (default: 1)
- `page_size`: Number of items per page (default: 20, max: 100)
- `sort_by`: Field to sort by (default varies by endpoint)
- `sort_order`: Sort order ('asc' or 'desc', default: 'desc')

## 7. Rate Limiting

All endpoints are subject to rate limiting:
- 100 requests per minute per IP
- 1000 requests per hour per authenticated user
- Exceeded requests return 429 status code with retry-after header