# Data Model: TaskApp Professional Styling and Authentication

## Overview
This document defines the data models for the TaskApp application, including entities, relationships, and validation rules.

## Entity: User
**Description**: Represents a registered user in the system

**Fields**:
- id: UUID (Primary Key, auto-generated)
- email: String (Required, unique, indexed, max 255 chars, valid email format)
- password_hash: String (Required, hashed using bcrypt or similar)
- created_at: DateTime (Auto-generated timestamp)
- updated_at: DateTime (Auto-generated timestamp, updated on changes)
- is_active: Boolean (Default: true)

**Validation Rules**:
- Email must be unique across all users
- Email must follow standard email format (user@domain.tld)
- Password must meet complexity requirements (min 8 chars, 1 uppercase, 1 lowercase, 1 number)
- User cannot be created with duplicate email

**Relationships**:
- One-to-Many: User → Task (user_id foreign key in Task)

## Entity: Task
**Description**: Represents a user's task with title, description, and status

**Fields**:
- id: UUID (Primary Key, auto-generated)
- title: String (Required, max 200 chars)
- description: Text (Optional, max 1000 chars)
- status: String (Required, enum: 'pending', 'in-progress', 'completed', default: 'pending')
- due_date: DateTime (Optional)
- created_at: DateTime (Auto-generated timestamp)
- updated_at: DateTime (Auto-generated timestamp, updated on changes)
- user_id: UUID (Foreign Key → User.id, required)

**Validation Rules**:
- Title must be between 1 and 200 characters
- Description, if provided, must be less than 1000 characters
- Status must be one of the allowed values ('pending', 'in-progress', 'completed')
- Due date, if provided, must be a future date
- Task must belong to an existing user

**Relationships**:
- Many-to-One: Task → User (via user_id foreign key)

## State Transitions

### Task Status Transitions
- pending → in-progress (when user starts working on task)
- in-progress → completed (when user finishes task)
- completed → pending (when user reopens task)
- in-progress → pending (when user stops working on task)

### User Activation Transitions
- inactive → active (when user verifies email or admin activates)
- active → inactive (when user deactivates account or admin deactivates)

## Indexes
- User.email: Unique index for fast lookup and constraint enforcement
- Task.user_id: Index for efficient user-based queries
- Task.status: Index for status-based filtering
- Task.due_date: Index for date-based queries
- Task.created_at: Index for chronological ordering

## Data Integrity Constraints
- Foreign Key Constraint: Task.user_id must reference an existing User.id
- Not Null Constraints: All required fields must have values
- Check Constraints: Status field limited to allowed values
- Unique Constraints: Email uniqueness enforced at database level

## Audit Trail Considerations
- created_at and updated_at fields automatically managed by the system
- Future extension: Consider adding a separate audit log table for tracking changes

## API Contract Implications
- User creation endpoint must hash passwords before storing
- Task creation endpoint must validate user ownership
- All date fields use ISO 8601 format for API communication
- UUIDs are represented as strings in API responses