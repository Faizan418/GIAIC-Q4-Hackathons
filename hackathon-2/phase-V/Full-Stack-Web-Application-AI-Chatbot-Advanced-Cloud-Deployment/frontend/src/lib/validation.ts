// frontend/lib/validation.ts
import { z } from 'zod';

// Task validation schema based on data-model.md requirements
export const taskSchema = z.object({
  id: z.string().uuid(),
  title: z.string().min(1, 'Title is required').max(200, 'Title must be 200 characters or less'),
  description: z.string().max(1000, 'Description must be 1000 characters or less').optional(),
  completed: z.boolean(),
  priority: z.enum(['low', 'medium', 'high'], {
    errorMap: () => ({ message: 'Priority must be low, medium, or high' })
  }),
  tags: z.array(z.string().trim().min(1).max(30)).max(10, 'Maximum 10 tags allowed'),
  dueDate: z.string().datetime().optional(),
  recurrence: z.object({
    type: z.enum(['none', 'daily', 'weekly', 'monthly']),
    interval: z.number().int().min(1).optional(),
    endDate: z.string().datetime().optional(),
    occurrences: z.number().int().min(1).optional()
  }).optional(),
  createdAt: z.string().datetime(),
  updatedAt: z.string().datetime(),
  completedAt: z.string().datetime().optional()
});

// User preferences validation schema
export const userPreferencesSchema = z.object({
  theme: z.enum(['light', 'dark', 'system']),
  notifications: z.object({
    enabled: z.boolean(),
    dueTimeHours: z.number().min(1).max(168), // 1 week in hours
    deadlineReminders: z.boolean()
  }),
  dashboard: z.object({
    defaultView: z.enum(['list', 'grid']),
    showCompleted: z.boolean(),
    defaultSort: z.enum(['dueDate', 'priority', 'title', 'created']),
    defaultSortOrder: z.enum(['asc', 'desc'])
  }),
  createdAt: z.string().datetime(),
  updatedAt: z.string().datetime()
});

// Simplified task schema for creation (without id and timestamps)
export const createTaskSchema = taskSchema.omit({
  id: true,
  createdAt: true,
  updatedAt: true,
  completedAt: true
}).extend({
  // Make some fields optional for creation
  completed: z.boolean().optional().default(false),
  tags: z.array(z.string().trim().min(1).max(30)).max(10, 'Maximum 10 tags allowed').optional().default([]),
  createdAt: z.string().datetime().optional(),
  updatedAt: z.string().datetime().optional(),
  completedAt: z.string().datetime().optional()
});

// Simplified task schema for updates (partial fields)
export const updateTaskSchema = createTaskSchema.partial();

// Validate a task object
export function validateTask(task: any) {
  return taskSchema.safeParse(task);
}

// Validate user preferences
export function validateUserPreferences(preferences: any) {
  return userPreferencesSchema.safeParse(preferences);
}

// Validate task creation data
export function validateCreateTask(task: any) {
  return createTaskSchema.safeParse(task);
}

// Validate task update data
export function validateUpdateTask(task: any) {
  return updateTaskSchema.safeParse(task);
}