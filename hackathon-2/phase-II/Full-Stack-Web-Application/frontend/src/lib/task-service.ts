import { taskApi } from './api';

// Define the shape of our input for creating/updating tasks
interface CreateTaskInput {
  title: string;
  description?: string;
  priority?: 'low' | 'medium' | 'high'; // Updated to match backend API
  type?: 'daily' | 'weekly' | 'monthly'; // Backend expects lowercase values
  dueDate?: string; // Using string format for API compatibility
}

interface UpdateTaskInput {
  title?: string;
  description?: string;
  status?: string; // Updated to match backend API (pending, completed, etc.)
  priority?: 'low' | 'medium' | 'high'; // Updated to match backend API
  type?: 'daily' | 'weekly' | 'monthly'; // Updated to match backend API
  due_date?: string; // Using string format for API compatibility
}

/**
 * Service for managing tasks with CRUD operations
 */
export class TaskService {
  /**
   * Get all tasks (public access)
   */
  static async getAll() {
    try {
      const tasks = await taskApi.getTasks();
      return tasks;
    } catch (error) {
      console.error('Error fetching tasks:', error);
      throw new Error('Failed to fetch tasks');
    }
  }

  /**
   * Get a single task by ID
   */
  static async getById(id: string) {
    try {
      const task = await taskApi.getTask(id);
      return task;
    } catch (error) {
      console.error('Error fetching task:', error);
      throw new Error('Failed to fetch task');
    }
  }

  /**
   * Create a new task
   */
  static async create(input: CreateTaskInput) {
    try {
      // Convert our input to match the backend API format
      const taskData = {
        title: input.title,
        description: input.description || '',
        status: 'pending', // Default to pending
        priority: input.priority || 'medium', // Default to medium
        type: (input.type ? input.type.toLowerCase() : 'daily') as 'daily' | 'weekly' | 'monthly', // Send as type field that backend expects
        due_date: input.dueDate,
      };

      const newTask = await taskApi.createTask(taskData);
      return newTask;
    } catch (error) {
      console.error('Error creating task:', error);
      throw new Error('Failed to create task');
    }
  }

  /**
   * Update an existing task
   */
  static async update(id: string, input: Partial<UpdateTaskInput>) {
    try {
      // Prepare update data, converting to match backend API format
      const updateData: any = {};
      if (input.title !== undefined) updateData.title = input.title;
      if (input.description !== undefined) updateData.description = input.description;
      if (input.status !== undefined) updateData.status = input.status;
      if (input.priority !== undefined) updateData.priority = input.priority;
      if (input.type !== undefined) updateData.type = input.type;
      if (input.due_date !== undefined) updateData.due_date = input.due_date;

      const updatedTask = await taskApi.updateTask(id, updateData);
      return updatedTask;
    } catch (error) {
      console.error('Error updating task:', error);
      throw new Error('Failed to update task');
    }
  }

  /**
   * Delete a task
   */
  static async delete(id: string) {
    try {
      await taskApi.deleteTask(id);
      return true;
    } catch (error) {
      console.error('Error deleting task:', error);
      throw new Error('Failed to delete task');
    }
  }

  /**
   * Toggle task completion status
   */
  static async toggleCompletion(id: string, completed: boolean) {
    try {
      // The backend toggle endpoint just flips the current status, no need to pass status
      const updatedTask = await taskApi.updateTaskCompletion(id);
      return updatedTask;
    } catch (error) {
      console.error('Error updating task completion:', error);
      throw new Error('Failed to update task completion status');
    }
  }
}