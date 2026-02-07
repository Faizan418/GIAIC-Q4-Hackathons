# Data Model: Full-Stack Todo App Integration

**Feature**: Full-Stack Todo App Integration
**Date**: 2026-01-12
**Status**: Complete

## Entity: User

**Description**: Represents an authenticated user with unique ID, manages their own tasks

**Fields**:
- `id`: Unique identifier (string, UUID format)
- `email`: User's email address (string, required, unique)
- `name`: User's display name (string, required)
- `created_at`: Timestamp when user account was created (datetime, required)

**Relationships**:
- One-to-many with Task entity (user has many tasks)

**Validation rules**:
- Email must be valid email format
- Name must be 1-100 characters
- ID must be unique across all users

## Entity: Task

**Description**: Represents a todo item with title, description, priority, tags, due date, recurrence pattern, completion status

**Fields**:
- `id`: Unique identifier (integer, auto-incrementing)
- `user_id`: Foreign key linking to User (string, required)
- `title`: Task title (string, 1-100 characters, required)
- `description`: Task description (string, optional, max 5000 characters)
- `completed`: Completion status (boolean, default false)
- `priority`: Task priority level (enum: high, medium, low, optional)
- `tags`: Task tags/labels (array of strings, optional)
- `due_date`: Deadline for task (datetime, optional)
- `recurrence`: Recurrence pattern (enum: none, daily, weekly, default none)
- `created_at`: Timestamp when task was created (datetime, required)
- `updated_at`: Timestamp when task was last updated (datetime, required)

**State transitions**:
- Created → Active (when task is created)
- Active → Completed (when task is marked as complete)
- Completed → Active (when task is marked as incomplete again)
- For recurring tasks: Completed → New instance created for next occurrence

**Validation rules**:
- Title must be 1-100 characters
- Description must be 5000 characters or less
- Priority must be one of the allowed values
- User_id must reference an existing user
- Due date must be in the future if provided

## Entity: JWT Token

**Description**: Authentication token that provides access to user-specific data and ensures data isolation

**Fields**:
- `token`: The JWT token string (string, required)
- `user_id`: ID of the user the token belongs to (string, required)
- `expires_at`: Expiration timestamp (datetime, required)
- `issued_at`: Issue timestamp (datetime, required)

**Validation rules**:
- Token must be properly formatted JWT
- User_id must reference an existing user
- Token must not be expired when used

## Entity: API Session

**Description**: Represents the connection between frontend and backend with proper authentication headers

**Fields**:
- `session_id`: Unique session identifier (string, required)
- `user_id`: ID of the user associated with session (string, required)
- `token`: Current JWT token for the session (string, required)
- `created_at`: Session creation timestamp (datetime, required)
- `last_activity_at`: Last activity timestamp (datetime, required)

**Validation rules**:
- Session must have valid, non-expired token
- User_id must match token's user_id
- Session must be refreshed before expiration

## Relationships

**User → Task** (One-to-Many)
- Each user can have multiple tasks
- Tasks are filtered by user_id to ensure data isolation
- When user is deleted, all their tasks are also deleted (cascade delete)

**User → JWT Token** (One-to-Many)
- Each user can have multiple tokens (for different devices/sessions)
- Tokens are validated to ensure they belong to the correct user

## Constraints

**Data Integrity**:
- All tasks must have a valid user_id referencing an existing user
- User data isolation is enforced by checking user_id in token matches user_id in URL/path
- All timestamps are stored in UTC

**Security**:
- User_id from JWT token must match the user_id in API request path
- No cross-user data access is allowed
- All sensitive operations require valid JWT token

## Indexes

**Required indexes for performance**:
- `tasks.user_id` - For filtering tasks by user
- `tasks.completed` - For filtering completed/incomplete tasks
- `tasks.due_date` - For sorting by due date
- `tasks.priority` - For filtering by priority
- `tasks.created_at` - For chronological ordering