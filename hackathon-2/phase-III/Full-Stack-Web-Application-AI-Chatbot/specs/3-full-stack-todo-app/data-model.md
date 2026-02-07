# Data Model: TASKAPP Professional Todo Application

## Overview
This document defines the data model for the TASKAPP professional todo application, including entities, relationships, validation rules, and state transitions based on the feature specification.

## Core Entities

### User Entity
Represents a registered user with email, name, and authentication credentials managed by Better Auth.

**Fields**:
- `id` (str): Unique identifier for the user (UUID or string from Better Auth)
- `email` (str): User's email address (unique, required)
- `name` (str): User's display name (required)
- `created_at` (datetime): Timestamp of account creation (auto-generated)

**Validation Rules**:
- Email must be valid email format (RFC 5322)
- Email must be unique across all users
- Name must be 1-100 characters
- All fields required at creation

**Relationships**:
- One-to-many with Task entity (user has many tasks)

### Task Entity
Represents a user's todo item with title, description, completion status, priority, tags, due date, and recurrence settings, associated with a specific user.

**Fields**:
- `id` (int): Primary key (auto-increment)
- `user_id` (str): Foreign key linking to User (required, indexed)
- `title` (str): Task title (required, 1-200 characters)
- `description` (str): Optional task description (max 1000 characters)
- `completed` (bool): Completion status (default: false, indexed)
- `priority` (enum): Priority level (values: "low", "medium", "high"; default: "medium")
- `tags` (List[str]): Array of tags for categorization (default: [])
- `due_date` (datetime): Optional deadline (nullable, timezone-aware)
- `recurrence` (enum): Recurrence pattern (values: "none", "daily", "weekly"; default: "none")
- `created_at` (datetime): Creation timestamp (auto-generated)
- `updated_at` (datetime): Last update timestamp (auto-generated, updates on change)

**Validation Rules**:
- Title must be 1-200 characters
- Description must be 0-1000 characters if provided
- User_id must reference an existing user
- Due date must be in the future if provided
- Priority must be one of the defined enum values
- Recurrence must be one of the defined enum values

**State Transitions**:
- Active → Completed: When user marks task as complete
- Completed → Active: When user unmarks task (if feature supported)
- With recurrence: When completed, system creates new instance according to recurrence pattern

**Indexing Strategy**:
- Primary: id
- Foreign key: user_id (critical for user isolation)
- Status: completed (for filtering completed/incomplete tasks)
- Date: due_date (for sorting and filtering by deadline)
- Composite: (user_id, completed) for efficient user-specific status queries

## Supporting Enums

### Priority Enum
Defines the priority levels for tasks.

**Values**:
- `LOW` = "low"
- `MEDIUM` = "medium"
- `HIGH` = "high"

### Recurrence Enum
Defines the recurrence patterns for tasks.

**Values**:
- `NONE` = "none"
- `DAILY` = "daily"
- `WEEKLY` = "weekly"

## Relationships

### User → Task (One-to-Many)
- One user can have multiple tasks
- Each task belongs to exactly one user
- Implemented via foreign key constraint: Task.user_id → User.id
- Critical for user isolation and data security

## Business Logic Requirements

### Recurring Tasks
When a task with recurrence is marked as complete:
1. Current task is marked as completed
2. New task is automatically created with:
   - Same title, description, priority, tags
   - New due_date calculated based on recurrence pattern:
     - Daily: previous due_date + 1 day
     - Weekly: previous due_date + 7 days
   - Completed status set to false
   - New creation timestamp
3. Limit future generations to prevent infinite chains (e.g., max 50 instances)

### User Isolation
- All queries must filter by user_id to ensure users only access their own data
- Authorization checks must verify that requested task belongs to authenticated user
- API endpoints should accept user_id in path to reinforce isolation

### Data Validation
- Server-side validation required for all inputs
- Client-side validation for UX improvement
- Sanitization of inputs to prevent injection attacks
- Proper timezone handling for due_date field

## Database Schema

### SQL Tables

```sql
-- Users table (managed by Better Auth)
CREATE TABLE users (
    id VARCHAR(36) PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    name VARCHAR(100) NOT NULL,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Indexes for users
CREATE UNIQUE INDEX idx_users_email ON users (email);

-- Tasks table
CREATE TABLE tasks (
    id SERIAL PRIMARY KEY,
    user_id VARCHAR(36) NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    title VARCHAR(200) NOT NULL,
    description TEXT,
    completed BOOLEAN DEFAULT FALSE,
    priority VARCHAR(10) DEFAULT 'medium',
    tags TEXT[],
    due_date TIMESTAMPTZ,
    recurrence VARCHAR(10) DEFAULT 'none',
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- Indexes for tasks
CREATE INDEX idx_tasks_user_id ON tasks (user_id);
CREATE INDEX idx_tasks_completed ON tasks (completed);
CREATE INDEX idx_tasks_due_date ON tasks (due_date);
CREATE INDEX idx_tasks_priority ON tasks (priority);
CREATE INDEX idx_tasks_user_completed ON tasks (user_id, completed);
CREATE INDEX idx_tasks_tags_gin ON tasks USING GIN (tags);
```

## API Representation

### Task Schema (JSON)
```json
{
  "id": 1,
  "user_id": "user-uuid-string",
  "title": "Task title",
  "description": "Optional description",
  "completed": false,
  "priority": "high",
  "tags": ["work", "urgent"],
  "due_date": "2026-01-15T10:00:00Z",
  "recurrence": "daily",
  "created_at": "2026-01-14T12:00:00Z",
  "updated_at": "2026-01-14T12:00:00Z"
}
```

## Security Considerations

### Data Access Control
- All database queries must include user_id filter
- Row-level security enforced via application logic
- API endpoints must validate that authenticated user owns requested resource

### Data Encryption
- Sensitive data encrypted at rest using database-level encryption
- Data in transit encrypted via TLS
- Authentication tokens stored securely

## Performance Considerations

### Query Optimization
- Proper indexing strategy implemented for common query patterns
- Pagination for large result sets
- Efficient filtering and sorting capabilities
- Caching for frequently accessed data

### Scalability
- Database schema designed to support horizontal scaling
- Connection pooling for efficient database access
- Asynchronous operations for I/O-bound tasks