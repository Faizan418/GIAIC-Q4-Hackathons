# Data Model: Backend Todo API

## Entities

### User Entity (referenced from Better Auth)
This entity is managed by Better Auth and referenced by our application.

**Fields:**
- `id`: string (primary key, unique identifier from Better Auth)
- `email`: string (unique, email address from Better Auth)
- `name`: string (user's name from Better Auth)
- `created_at`: datetime (timestamp when user was created in Better Auth)

**Relationships:**
- One-to-many relationship with Task entity (one user can have many tasks)

### Task Entity
This is the main entity for our todo application.

**Fields:**
- `id`: integer (primary key, auto-increment)
- `user_id`: string (foreign key to User.id, required)
- `title`: string (required, 1-100 characters)
- `description`: string (optional, max 5000 characters)
- `completed`: boolean (default False)
- `priority`: enum (optional, values: 'high', 'medium', 'low')
- `tags`: List[string] (optional, array of tag strings)
- `due_date`: datetime (optional, stored in UTC)
- `recurrence`: enum (default 'none', values: 'none', 'daily', 'weekly')
- `created_at`: datetime (auto-set when created)
- `updated_at`: datetime (auto-updated when modified)

**Relationships:**
- Many-to-one relationship with User entity (many tasks belong to one user)

## Database Schema

### Tables

#### tasks table
```
id: INTEGER PRIMARY KEY
user_id: STRING NOT NULL (foreign key to users.id)
title: STRING(100) NOT NULL
description: TEXT
completed: BOOLEAN DEFAULT FALSE
priority: VARCHAR(10) CHECK (priority IN ('high', 'medium', 'low'))
tags: JSONB DEFAULT '[]' (array of strings)
due_date: TIMESTAMP WITH TIME ZONE
recurrence: VARCHAR(10) DEFAULT 'none' CHECK (recurrence IN ('none', 'daily', 'weekly'))
created_at: TIMESTAMP WITH TIME ZONE DEFAULT NOW()
updated_at: TIMESTAMP WITH TIME ZONE DEFAULT NOW()
```

**Indexes:**
- `idx_tasks_user_id`: INDEX ON (user_id) - for fast user-based filtering
- `idx_tasks_completed`: INDEX ON (completed) - for status-based queries
- `idx_tasks_due_date`: INDEX ON (due_date) - for date-based queries
- `idx_tasks_priority`: INDEX ON (priority) - for priority-based queries

## Relationships

### User → Task
- One-to-many relationship
- Foreign key constraint: tasks.user_id → users.id
- Cascading behavior: When user is deleted, related tasks should also be deleted (though in our case, users are managed by Better Auth)

## Validation Rules

### Task Validation
- Title: Required, 1-100 characters
- Description: Optional, max 5000 characters
- Priority: Optional, must be one of 'high', 'medium', 'low'
- Tags: Optional, array of strings
- Due date: Optional, must be valid datetime
- Recurrence: Optional, must be one of 'none', 'daily', 'weekly'

### Business Rules
- All tasks must be associated with a valid user
- Users can only access their own tasks
- Recurring tasks generate new tasks when completed
- Completed recurring tasks create new instances with updated due dates

## State Transitions

### Task State Transitions
- `incomplete` → `completed`: When user marks task as complete
- `completed` → `incomplete`: When user unmarks task as complete
- `completed` → `incomplete` + `new_task_created`: When recurring task is completed

## Constraints

### Database Constraints
- NOT NULL constraints on required fields (id, user_id, title)
- Check constraints on enum fields (priority, recurrence)
- Foreign key constraint linking user_id to users table

### Application Constraints
- User isolation: Users can only access tasks associated with their user_id
- Rate limiting: Maximum 100 requests per IP per hour
- Time zone handling: All due dates stored in UTC, converted for display
- Deletion: Hard delete for tasks (permanent removal)