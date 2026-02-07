
const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

/**
 * Generic API request function without authentication
 */
async function apiRequest<T>(
  endpoint: string,
  options: RequestInit = {}
): Promise<T> {
  const headers = {
    'Content-Type': 'application/json',
    ...options.headers,
  };

  const response = await fetch(`${API_BASE_URL}${endpoint}`, {
    ...options,
    headers,
    // Remove credentials since we're not using auth
    credentials: 'omit',
    // Prevent caching to ensure fresh data on page refresh
    cache: 'no-store',
  });

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
  }

  // Handle responses without body (e.g., 204 No Content)
  if (response.status === 204) {
    return undefined as T;
  }

  return response.json();
}

/**
 * Task API functions
 */
export const taskApi = {
  /**
   * Get all tasks for the authenticated user
   */
  getTasks: async (params?: {
    completed?: boolean;
    priority?: string;
    due_date?: string;
    search?: string;
    sort?: string;
    order?: 'asc' | 'desc';
    limit?: number;
    offset?: number;
  }) => {
    const queryParams = new URLSearchParams();

    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined && value !== null) {
          queryParams.append(key, String(value));
        }
      });
    }

    const queryString = queryParams.toString();
    // Updated to match the new backend route pattern (without user_id in path)
    const endpoint = `/api/tasks${queryString ? `?${queryString}` : ''}`;

    const response = await apiRequest<TaskListResponse>(endpoint);
    // Extract tasks from the response.data.tasks structure
    return response.data?.tasks || [];
  },

  /**
   * Create a new task
   */
  createTask: async (taskData: Omit<Task, 'id' | 'user_id' | 'created_at' | 'updated_at'>) => {
    const response = await apiRequest<{ success: boolean; data: { task: Task } }>('/api/tasks', {
      method: 'POST',
      body: JSON.stringify(taskData),
    });
    return response.data.task; // Extract task from response.data.task
  },

  /**
   * Get a specific task
   */
  getTask: async (taskId: string) => {
    const response = await apiRequest<{ success: boolean; data: { task: Task } }>(`/api/tasks/${taskId}`);
    return response.data.task; // Extract task from response.data.task
  },

  /**
   * Update a task
   */
  updateTask: async (taskId: string, taskData: Partial<Task>) => {
    const response = await apiRequest<{ success: boolean; data: { task: Task } }>(`/api/tasks/${taskId}`, {
      method: 'PUT',
      body: JSON.stringify(taskData),
    });
    return response.data.task; // Extract task from response.data.task
  },

  /**
   * Delete a task
   */
  deleteTask: async (taskId: string) => {
    return apiRequest<{ success: boolean; message: string }>(`/api/tasks/${taskId}`, {
      method: 'DELETE',
    });
  },

  /**
   * Update task completion status
   */
  updateTaskCompletion: async (taskId: string) => {
    const response = await apiRequest<{ success: boolean; data: { task: Task } }>(`/api/tasks/${taskId}/complete`, {
      method: 'PATCH',
    });
    return response.data.task; // Extract task from response.data.task
  },
};

export const dashboardApi = {
  /**
   * Get dashboard statistics
   */
  getStats: async () => {
    // Public access only - no authentication required
    const response = await fetch(`${API_BASE_URL}/api/dashboard/stats`, {
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },
};

// Define TypeScript interfaces for our API models
export interface Task {
  id: string;  // Changed from number to string to match backend UUID
  user_id: string;
  title: string;
  description?: string;
  status: string; // Can be "pending", "completed", etc. - matches backend Task model
  priority: 'low' | 'medium' | 'high';
  type: 'daily' | 'weekly' | 'monthly'; // Matches backend Task model
  due_date?: string; // ISO string format
  created_at: string; // ISO string format
  updated_at: string; // ISO string format
}

export interface TaskListResponse {
  success: boolean;
  data: {
    tasks: Task[];
    total: number;
    skip: number;
    limit: number;
  };
  message?: string;
}