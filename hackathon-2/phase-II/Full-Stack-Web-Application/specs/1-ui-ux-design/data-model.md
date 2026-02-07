# Data Model: TASKAPP Frontend

## 1. User Entity

### Interface Definition
```typescript
interface User {
  id: string;
  email: string;
  name?: string;
  createdAt: string; // ISO 8601 datetime
  updatedAt: string; // ISO 8601 datetime
  preferences: UserPreferences;
}
```

### Validation Rules
- `id`: Required, unique identifier
- `email`: Required, valid email format
- `name`: Optional, max 100 characters
- `createdAt`: Required, server-generated timestamp
- `updatedAt`: Required, server-generated timestamp
- `preferences`: Required, contains user-specific settings

## 2. Task Entity

### Interface Definition
```typescript
interface Task {
  id: string;
  userId: string; // Foreign key to User
  title: string;
  description?: string;
  completed: boolean;
  priority: 'low' | 'medium' | 'high';
  tags: string[];
  dueDate?: string; // ISO 8601 date
  recurrence: 'none' | 'daily' | 'weekly' | 'monthly';
  createdAt: string; // ISO 8601 datetime
  updatedAt: string; // ISO 8601 datetime
}
```

### Validation Rules
- `id`: Required, unique identifier
- `userId`: Required, must correspond to an existing user
- `title`: Required, 1-200 characters
- `description`: Optional, max 1000 characters
- `completed`: Required, boolean value
- `priority`: Required, one of 'low', 'medium', 'high'
- `tags`: Optional, array of strings, max 10 tags
- `dueDate`: Optional, valid ISO 8601 date format
- `recurrence`: Required, one of 'none', 'daily', 'weekly', 'monthly'
- `createdAt`: Required, server-generated timestamp
- `updatedAt`: Required, server-generated timestamp

### State Transitions
- `completed: false` → `completed: true` (mark as complete)
- `completed: true` → `completed: false` (mark as incomplete)

## 3. ThemePreference Entity

### Interface Definition
```typescript
interface ThemePreference {
  userId: string; // Foreign key to User
  theme: 'dark' | 'light' | 'system';
  updatedAt: string; // ISO 8601 datetime
}
```

### Validation Rules
- `userId`: Required, must correspond to an existing user
- `theme`: Required, one of 'dark', 'light', 'system'
- `updatedAt`: Required, server-generated timestamp

## 4. NotificationPreference Entity

### Interface Definition
```typescript
interface NotificationPreference {
  userId: string; // Foreign key to User
  emailNotifications: boolean;
  emailFrequency: 'immediate' | 'daily' | 'weekly';
  deadlineReminders: boolean;
  deadlineHoursBefore: number; // Hours before deadline to send notification
  updatedAt: string; // ISO 8601 datetime
}
```

### Validation Rules
- `userId`: Required, must correspond to an existing user
- `emailNotifications`: Required, boolean value
- `emailFrequency`: Required, one of 'immediate', 'daily', 'weekly'
- `deadlineReminders`: Required, boolean value
- `deadlineHoursBefore`: Required, number between 1 and 168 (max 1 week)
- `updatedAt`: Required, server-generated timestamp

## 5. API Response Structures

### Authentication Responses
```typescript
interface SignInResponse {
  success: boolean;
  user?: User;
  error?: string;
  token?: string;
}

interface SignUpResponse {
  success: boolean;
  user?: User;
  error?: string;
  token?: string;
}
```

### Task API Responses
```typescript
interface TaskListResponse {
  success: boolean;
  tasks: Task[];
  pagination?: {
    page: number;
    pageSize: number;
    total: number;
  };
  error?: string;
}

interface TaskResponse {
  success: boolean;
  task?: Task;
  error?: string;
}
```

### User Preferences Responses
```typescript
interface UserPreferencesResponse {
  success: boolean;
  preferences?: {
    theme: ThemePreference;
    notifications: NotificationPreference;
  };
  error?: string;
}

interface UpdatePreferencesResponse {
  success: boolean;
  preferences?: {
    theme: ThemePreference;
    notifications: NotificationPreference;
  };
  error?: string;
}
```

## 6. Form Data Structures

### Task Creation Form
```typescript
interface TaskFormData {
  title: string;
  description?: string;
  priority: 'low' | 'medium' | 'high';
  tags: string[];
  dueDate?: string;
  recurrence: 'none' | 'daily' | 'weekly' | 'monthly';
}
```

### User Registration Form
```typescript
interface RegistrationFormData {
  name: string;
  email: string;
  password: string;
  confirmPassword: string;
}
```

### User Login Form
```typescript
interface LoginFormData {
  email: string;
  password: string;
}
```

## 7. Validation Schemas

### Task Validation
- Title: Required, 1-200 characters
- Description: Optional, max 1000 characters
- Priority: Required, must be 'low', 'medium', or 'high'
- Tags: Optional, max 10 tags, each 1-30 characters
- Due Date: Optional, must be valid date if provided
- Recurrence: Required, must be 'none', 'daily', 'weekly', or 'monthly'

### User Validation
- Name: Optional, max 100 characters
- Email: Required, valid email format
- Password: Required, min 8 characters with complexity requirements

### Notification Preferences Validation
- Email Frequency: Required, must be 'immediate', 'daily', or 'weekly'
- Deadline Hours Before: Required, between 1 and 168 (hours in a week)