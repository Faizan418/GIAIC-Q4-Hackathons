# Data Model: Frontend Todo App

## User Entity

### Fields
- **id** (string, required)
  - Unique identifier for the user
  - Format: UUID or similar unique string
- **email** (string, required)
  - User's email address for authentication
  - Format: Valid email address
  - Validation: Must be unique across all users
- **name** (string, required)
  - User's display name
  - Format: 1-100 characters
- **createdAt** (Date, required)
  - Timestamp when the user account was created
- **updatedAt** (Date, required)
  - Timestamp when the user account was last updated

### Relationships
- One User has many Tasks (user.tasks)

## Task Entity

### Fields
- **id** (string, required)
  - Unique identifier for the task
  - Format: UUID or similar unique string
- **title** (string, required)
  - Task title/description
  - Format: 1-200 characters
  - Validation: Required field, minimum 1 character, maximum 200 characters
- **description** (string, optional)
  - Detailed description of the task
  - Format: Up to 1000 characters
  - Validation: Optional, maximum 1000 characters if provided
- **status** (string, required)
  - Current status of the task
  - Values: 'pending', 'completed'
  - Default: 'pending'
- **priority** (string, required)
  - Priority level of the task
  - Values: 'high', 'medium', 'low'
  - Default: 'medium'
- **tags** (string array, optional)
  - Category labels for the task
  - Format: Array of strings
  - Validation: Each tag should be 1-50 characters
- **dueDate** (Date or null, optional)
  - Deadline for completing the task
  - Format: ISO 8601 date string or null
  - Validation: Optional, can be null
- **recurrencePattern** (string, required)
  - Pattern for recurring tasks
  - Values: 'none', 'daily', 'weekly'
  - Default: 'none'
- **userId** (string, required)
  - Foreign key linking to the User who owns this task
  - Format: Matches the id of a valid User
- **createdAt** (Date, required)
  - Timestamp when the task was created
- **updatedAt** (Date, required)
  - Timestamp when the task was last updated

### Validation Rules
1. **Title validation**:
   - Required field
   - Minimum length: 1 character
   - Maximum length: 200 characters

2. **Description validation**:
   - Optional field
   - Maximum length: 1000 characters

3. **Status validation**:
   - Required field
   - Must be one of: 'pending', 'completed'

4. **Priority validation**:
   - Required field
   - Must be one of: 'high', 'medium', 'low'

5. **Tags validation**:
   - Optional field
   - Each tag must be 1-50 characters
   - Maximum of 10 tags per task

6. **Due date validation**:
   - Optional field
   - If provided, must be a valid date

7. **Recurrence pattern validation**:
   - Required field
   - Must be one of: 'none', 'daily', 'weekly'

### State Transitions
- **Status**: Can transition between 'pending' and 'completed' based on user action
- **Priority**: Can be updated from any priority level to any other
- **Tags**: Can be added, removed, or modified
- **Due date**: Can be set, updated, or cleared
- **Recurrence**: Can be changed from any pattern to any other pattern

## Frontend-Specific Considerations

### TypeScript Interfaces

```typescript
interface User {
  id: string;
  email: string;
  name: string;
  createdAt: Date;
  updatedAt: Date;
}

interface Task {
  id: string;
  title: string;
  description?: string;
  status: 'pending' | 'completed';
  priority: 'high' | 'medium' | 'low';
  tags: string[];
  dueDate: Date | null;
  recurrencePattern: 'none' | 'daily' | 'weekly';
  userId: string;
  createdAt: Date;
  updatedAt: Date;
}
```

### Form Validation Schema (Zod)

```typescript
import { z } from 'zod';

const taskSchema = z.object({
  title: z.string().min(1, 'Title is required').max(200, 'Title must be less than 200 characters'),
  description: z.string().max(1000, 'Description must be less than 1000 characters').optional().nullable(),
  status: z.enum(['pending', 'completed'], { required_error: 'Status is required' }),
  priority: z.enum(['high', 'medium', 'low'], { required_error: 'Priority is required' }),
  tags: z.array(z.string().min(1).max(50)).max(10, 'Maximum 10 tags allowed').optional(),
  dueDate: z.union([z.string().datetime(), z.null()]).optional(),
  recurrencePattern: z.enum(['none', 'daily', 'weekly'], { required_error: 'Recurrence pattern is required' })
});

type TaskFormValues = z.infer<typeof taskSchema>;
```

### UI State Considerations
- Loading states for data fetching operations
- Error states for failed operations
- Optimistic update states for immediate UI feedback
- Form validation states for user input
- Search/filter state for task organization