/**
 * Task type definition that matches the Neon PostgreSQL schema
 */

export interface Task {
  id: string;
  title: string;
  description?: string;
  is_completed: boolean;
  priority: 'LOW' | 'MEDIUM' | 'HIGH';
  due_date?: string; // ISO string format
  recurrence_pattern?: string; // e.g. 'daily', 'weekly', 'monthly'
  created_at: string; // ISO string format
  updated_at: string; // ISO string format
  user_id: string;
  order_index: string;
}