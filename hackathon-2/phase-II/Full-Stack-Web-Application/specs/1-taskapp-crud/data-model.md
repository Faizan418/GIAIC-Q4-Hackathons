# TASKAPP Data Model

## 1. Core Entities

### 1.1 Task Entity

**Definition**: The Task entity represents individual tasks that users can create, manage, and track.

**Fields**:
- `id` (UUID, Primary Key)
  - Auto-generated using gen_random_uuid()
  - Required for all operations
  - Immutable after creation

- `title` (TEXT, Required)
  - Minimum length: 1 character
  - Maximum length: 200 characters
  - Must not be empty or whitespace only

- `description` (TEXT, Optional)
  - Maximum length: 1000 characters
  - Can be null or empty

- `completed` (BOOLEAN)
  - Default value: false
  - Represents task completion status

- `priority` (TEXT with constraint)
  - Allowed values: 'low', 'medium', 'high'
  - Default value: 'medium'
  - Must be one of the allowed values

- `category` (TEXT, Optional)
  - Free-form text field
  - Maximum length: 100 characters
  - Can be used for grouping tasks

- `type` (TEXT with constraint)
  - Allowed values: 'daily', 'weekly', 'monthly'
  - Required field
  - Determines recurrence pattern

- `dueDate` (TIMESTAMP, Optional)
  - Stores the due date and time for the task
  - Can be null if no due date is set

- `createdAt` (TIMESTAMP)
  - Auto-generated using NOW()
  - Immutable after creation
  - Tracks when the task was created

- `updatedAt` (TIMESTAMP)
  - Auto-generated using NOW()
  - Updates on every modification
  - Tracks last update time

- `completedAt` (TIMESTAMP, Optional)
  - Set when the task is marked as completed
  - Null when task is not completed

### 1.2 Database Schema (PostgreSQL)

```sql
CREATE TABLE tasks (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  title TEXT NOT NULL CHECK (length(title) >= 1 AND length(title) <= 200),
  description TEXT,
  completed BOOLEAN DEFAULT FALSE,
  priority TEXT CHECK (priority IN ('low','medium','high')) DEFAULT 'medium',
  category TEXT,
  type TEXT CHECK (type IN ('daily','weekly','monthly')) NOT NULL,
  due_date TIMESTAMP,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),
  completed_at TIMESTAMP
);
```

## 2. Validation Rules

### 2.1 Field-Level Validation
- `title`: Required, 1-200 characters, not just whitespace
- `description`: Optional, max 1000 characters
- `priority`: Required enum value ('low', 'medium', 'high')
- `type`: Required enum value ('daily', 'weekly', 'monthly')
- `category`: Optional, max 100 characters
- `dueDate`: If provided, must be a valid date/time

### 2.2 Business Rule Validation
- A task cannot be marked as completed before its creation date
- The `updatedAt` field must always be greater than or equal to `createdAt`
- The `completedAt` field must be null if `completed` is false

## 3. Indexes

### 3.1 Recommended Indexes
```sql
-- Index for common filtering operations
CREATE INDEX idx_tasks_completed ON tasks (completed);
CREATE INDEX idx_tasks_priority ON tasks (priority);
CREATE INDEX idx_tasks_type ON tasks (type);
CREATE INDEX idx_tasks_due_date ON tasks (due_date);
CREATE INDEX idx_tasks_created_at ON tasks (created_at);
CREATE INDEX idx_tasks_updated_at ON tasks (updated_at);
```

## 4. Relationships

### 4.1 Current Implementation
- Tasks are independent entities (no direct relationships)
- All fields are contained within the task entity

### 4.2 Future Extensions
- Potential relationship to a User entity for multi-user support
- Potential relationship to a Category entity for better categorization
- Potential relationship to a RecurrenceRule entity for advanced recurrence

## 5. State Transitions

### 5.1 Task Lifecycle States
```
[CREATED] -> [COMPLETED] -> [ARCHIVED/OPTIONAL_RECUR]
```

### 5.2 State Transition Rules
- A task moves from CREATED to COMPLETED when marked as complete
- When completed, the `completedAt` field is set to current timestamp
- The `completed` field is updated to true
- If recurrence is enabled, a new instance may be created based on the type

## 6. Data Integrity Constraints

### 6.1 Check Constraints
- Title length constraint: 1-200 characters
- Priority enum constraint: 'low', 'medium', 'high'
- Type enum constraint: 'daily', 'weekly', 'monthly'
- Description length constraint: max 1000 characters

### 6.2 Trigger Constraints
- `updatedAt` automatically updates on row modification
- `completedAt` is set when `completed` changes from false to true
- `completedAt` is cleared when `completed` changes from true to false

## 7. API Mapping

### 7.1 Request/Response Mapping
```
API Field        | Database Column | Validation
----------------|-----------------|------------------
id              | id              | UUID format
title           | title           | Required, 1-200 chars
description     | description     | Optional, max 1000 chars
completed       | completed       | Boolean
priority        | priority        | Enum: low,medium,high
category        | category        | Optional, max 100 chars
type            | type            | Enum: daily,weekly,monthly
dueDate         | due_date        | ISO 8601 datetime format
createdAt       | created_at      | Read-only, server-generated
updatedAt       | updated_at      | Read-only, server-generated
completedAt     | completed_at    | Read-only, server-generated
```

## 8. Migration Strategy

### 8.1 Initial Migration
- Create the tasks table with all specified constraints
- Enable UUID extension in PostgreSQL if not already enabled
- Create recommended indexes for performance

### 8.2 Future Migration Considerations
- Addition of user_id column for multi-user support
- Addition of recurrence_rule_id for advanced recurrence
- Partitioning strategy for large datasets