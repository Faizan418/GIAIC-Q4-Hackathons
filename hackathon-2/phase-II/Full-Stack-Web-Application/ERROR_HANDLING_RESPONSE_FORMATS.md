# Error Handling & Response Formats: Todo Application API

## Overview
This document details the error handling patterns, response format standards, and error classification system used throughout the Todo application API. It ensures consistent behavior across all endpoints and provides clear guidance for client applications on how to handle various error scenarios.

## Response Format Standards

### Success Response Format
All successful API responses follow a consistent wrapper pattern:

```json
{
  "success": true,
  "data": { ... },
  "message": "Optional success message"
}
```

**Fields**:
- `success`: Boolean indicating operation success (always `true` for successful operations)
- `data`: Contains the primary response payload (can be an object, array, or null)
- `message`: Optional string with additional success information

**Example**:
```json
{
  "success": true,
  "data": {
    "id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
    "title": "Sample Task",
    "completed": false
  },
  "message": "Task created successfully"
}
```

### Error Response Format
All error responses follow a consistent error wrapper pattern:

```json
{
  "success": false,
  "error": {
    "type": "ErrorType",
    "message": "Error description",
    "details": "Additional error details (optional)"
  }
}
```

**Fields**:
- `success`: Boolean indicating operation failure (always `false` for error responses)
- `error.type`: String identifying the error category
- `error.message`: Human-readable error description
- `error.details`: Optional field with specific error details (e.g., validation errors)

## HTTP Status Code Mapping

### Success Status Codes
| Code | Description | Usage |
|------|-------------|-------|
| 200 | OK | Standard success response for GET, PUT, PATCH operations |
| 201 | Created | Resource successfully created (typically POST operations) |

### Client Error Status Codes
| Code | Description | Error Type | Message Format |
|------|-------------|------------|----------------|
| 400 | Bad Request | `BadRequest` | General malformed request |
| 401 | Unauthorized | `AuthenticationError` | Invalid or missing authentication |
| 403 | Forbidden | `AuthorizationError` | Insufficient permissions |
| 404 | Not Found | `NotFoundError` | Requested resource does not exist |
| 422 | Unprocessable Entity | `ValidationError` | Request validation failed |
| 429 | Too Many Requests | `RateLimitError` | Rate limit exceeded |

### Server Error Status Codes
| Code | Description | Error Type | Message Format |
|------|-------------|------------|----------------|
| 500 | Internal Server Error | `ServerError` | Unexpected server error |

## Error Categories & Handling

### 1. ValidationError (422)
**Trigger**: Request body or query parameter validation fails

**Response Format**:
```json
{
  "success": false,
  "error": {
    "type": "ValidationError",
    "message": "Request validation failed",
    "details": [
      {
        "loc": ["body", "title"],
        "msg": "Field required",
        "type": "missing"
      }
    ]
  }
}
```

**Common Scenarios**:
- Missing required fields
- Invalid data types
- Field length violations
- Invalid enum values
- Malformed email addresses
- Invalid date formats

**Example**:
```json
{
  "success": false,
  "error": {
    "type": "ValidationError",
    "message": "Request validation failed",
    "details": [
      {
        "loc": ["body", "title"],
        "msg": "String should have at least 1 character",
        "type": "string_too_short"
      },
      {
        "loc": ["body", "due_date"],
        "msg": "Invalid datetime format",
        "type": "datetime_parsing"
      }
    ]
  }
}
```

### 2. AuthenticationError (401)
**Trigger**: Invalid, expired, or missing authentication token

**Response Format**:
```json
{
  "success": false,
  "error": {
    "type": "AuthenticationError",
    "message": "Invalid or expired authentication token"
  }
}
```

**Common Scenarios**:
- Missing Authorization header
- Invalid JWT token format
- Expired token
- Tampered token signature
- Invalid token issuer

### 3. AuthorizationError (403)
**Trigger**: Valid authentication but insufficient permissions

**Response Format**:
```json
{
  "success": false,
  "error": {
    "type": "AuthorizationError",
    "message": "Insufficient permissions to access this resource"
  }
}
```

**Common Scenarios**:
- Attempting to access another user's resources
- Insufficient role-based permissions
- Resource ownership mismatch

### 4. NotFoundError (404)
**Trigger**: Requested resource does not exist

**Response Format**:
```json
{
  "success": false,
  "error": {
    "type": "NotFoundError",
    "message": "Resource not found"
  }
}
```

**Common Scenarios**:
- Non-existent task ID
- Non-existent user
- Deleted resource

### 5. BadRequestError (400)
**Trigger**: Malformed request that cannot be processed

**Response Format**:
```json
{
  "success": false,
  "error": {
    "type": "BadRequestError",
    "message": "Malformed request"
  }
}
```

**Common Scenarios**:
- Invalid JSON format
- Unsupported media type
- Conflicting parameters

### 6. RateLimitError (429)
**Trigger**: Request rate limit exceeded

**Response Format**:
```json
{
  "success": false,
  "error": {
    "type": "RateLimitError",
    "message": "Rate limit exceeded. Please try again later."
  }
}
```

**Configuration**:
- Limit: 100 requests per IP per hour
- Reset window: 1 hour from first request

### 7. ServerError (500)
**Trigger**: Unexpected server error

**Response Format**:
```json
{
  "success": false,
  "error": {
    "type": "ServerError",
    "message": "An unexpected error occurred"
  }
}
```

**Common Scenarios**:
- Database connection failures
- Internal server exceptions
- Third-party service failures

## Specific Error Messages

### Authentication-Related Errors
| Scenario | Status | Message |
|----------|--------|---------|
| Invalid credentials | 401 | Incorrect email or password |
| Already registered | 400 | User with this email already exists |
| Token expired | 401 | Authentication token has expired |
| Invalid token | 401 | Invalid authentication token |

### Task-Related Errors
| Scenario | Status | Message |
|----------|--------|---------|
| Task not found | 404 | Task not found |
| Invalid task ID | 422 | Invalid task ID format |
| Unauthorized task access | 403 | You don't have permission to access this task |
| Invalid recurrence value | 422 | Recurrence must be one of: none, daily, weekly |
| Invalid priority value | 422 | Priority must be one of: high, medium, low |

### Validation Error Details
Validation errors include detailed information about what went wrong:

**Field Requirements**:
- `missing`: Required field not provided
- `string_too_short`: String shorter than minimum length
- `string_too_long`: String longer than maximum length
- `value_error`: General value validation failure
- `type_error`: Wrong data type provided

**Example Validation Error**:
```json
{
  "success": false,
  "error": {
    "type": "ValidationError",
    "message": "Request validation failed",
    "details": [
      {
        "loc": ["body", "title"],
        "msg": "String should have at least 1 character",
        "type": "string_too_short"
      },
      {
        "loc": ["body", "priority"],
        "msg": "Input should be 'high', 'medium' or 'low'",
        "type": "enum",
        "ctx": {
          "expected": "'high', 'medium' or 'low'"
        }
      }
    ]
  }
}
```

## Exception Handler Implementation

### Global Exception Handlers
The API implements global exception handlers in `backend/src/main.py`:

#### HTTP Exception Handler
```python
@app.exception_handler(StarletteHTTPException)
async def custom_http_exception_handler(request, exc):
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "success": False,
            "error": {
                "type": "HTTPException",
                "message": exc.detail
            }
        }
    )
```

#### Validation Exception Handler
```python
@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request, exc):
    return JSONResponse(
        status_code=422,
        content={
            "success": False,
            "error": {
                "type": "ValidationError",
                "details": exc.errors()
            }
        }
    )
```

## Client-Side Error Handling Guide

### Recommended Error Handling Pattern
```javascript
try {
  const response = await fetch('/api/tasks/', {
    headers: {
      'Authorization': `Bearer ${token}`,
      'Content-Type': 'application/json'
    }
  });

  const data = await response.json();

  if (!response.ok) {
    // Handle error response
    console.error('API Error:', data.error);
    throw new Error(data.error.message);
  }

  // Handle success response
  console.log('Success:', data.data);
} catch (error) {
  // Handle network errors or other exceptions
  console.error('Network Error:', error);
}
```

### Error Type Detection
Clients should detect error types by checking both the HTTP status code and the `success` field in the response:

```javascript
if (response.status >= 400) {
  if (data.success === false) {
    // API error with structured response
    switch (data.error.type) {
      case 'AuthenticationError':
        // Redirect to login
        break;
      case 'ValidationError':
        // Display validation messages
        break;
      default:
        // Show generic error message
    }
  } else {
    // Unexpected error format
  }
}
```

## Logging & Monitoring

### Error Logging Format
All errors are logged with consistent format including:
- Timestamp
- Request method and URL
- User ID (when available)
- Error type and message
- Stack trace (server errors only)

### Error Metrics
- Error rate by endpoint
- Error rate by type
- Daily error volume
- Error resolution time

## Testing Error Scenarios

### Required Error Tests
- Invalid authentication handling
- Missing required fields
- Invalid data types
- Boundary value testing
- Rate limit enforcement
- Resource not found scenarios