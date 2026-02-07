import { Task as ApiTask } from './api';
import { Task as UiTask } from '@/types/task';

// Type mapping functions
function uiTaskToApiTask(uiTask: Omit<UiTask, 'id' | 'createdAt' | 'updatedAt' | 'userId'>): Omit<ApiTask, 'id' | 'user_id' | 'created_at' | 'updated_at'> {
  return {
    title: uiTask.title,
    description: uiTask.description,
    status: uiTask.is_completed ? 'completed' : 'pending',
    priority: uiTask.priority.toLowerCase() as 'low' | 'medium' | 'high',
    type: 'daily', // Default type for new tasks
    due_date: uiTask.due_date,
  };
}

function apiTaskToUiTask(apiTask: ApiTask): UiTask {
  return {
    id: apiTask.id,
    title: apiTask.title,
    description: apiTask.description,
    is_completed: apiTask.status === 'completed',
    priority: apiTask.priority.toUpperCase() as 'LOW' | 'MEDIUM' | 'HIGH',
    due_date: apiTask.due_date,
    created_at: apiTask.created_at,
    updated_at: apiTask.updated_at,
    user_id: apiTask.user_id,
    order_index: '0',
  };
}

interface User {
  id: string;
  email: string;
  name?: string;
}

interface AuthResponse {
  access_token: string;
  token_type: string;
}

interface RegisterData {
  email: string;
  password: string;
  name: string;
}

interface LoginData {
  email: string;
  password: string;
}

const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

let authToken: string | null = null;

export const authApi = {
  // Store token in memory
  setAuthToken(token: string | null) {
    authToken = token;
    if (token) {
      localStorage.setItem('authToken', token);
    } else {
      localStorage.removeItem('authToken');
    }
  },

  // Get token from memory or localStorage
  getAuthToken(): string | null {
    if (authToken) return authToken;
    return typeof window !== 'undefined' ? localStorage.getItem('authToken') : null;
  },

  // Register a new user
  async register(userData: RegisterData): Promise<AuthResponse> {
    const response = await fetch(`${API_BASE_URL}/api/auth/register`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(userData),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    const result = await response.json();
    this.setAuthToken(result.access_token);
    return result;
  },

  // Login user
  async login(credentials: LoginData): Promise<AuthResponse> {
    const response = await fetch(`${API_BASE_URL}/api/auth/login`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(credentials),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    const result = await response.json();
    this.setAuthToken(result.access_token);
    return result;
  },

  // Get current user info
  async getCurrentUser(): Promise<User> {
    const token = this.getAuthToken();
    if (!token) {
      throw new Error('User not authenticated');
    }

    const response = await fetch(`${API_BASE_URL}/api/auth/me`, {
      headers: {
        'Authorization': `Bearer ${token}`,
      },
    });

    if (!response.ok) {
      if (response.status === 401) {
        this.setAuthToken(null); // Clear invalid token
      }
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // Logout user
  async logout() {
    this.setAuthToken(null);
  },

  // Check if user is authenticated
  isAuthenticated(): boolean {
    return this.getAuthToken() !== null;
  },
};

// Enhanced task API to use auth token
export const taskApiWithAuth = {
  // Get all tasks for authenticated user
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
    const token = authApi.getAuthToken();
    if (!token) {
      throw new Error('User not authenticated');
    }

    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined && value !== null) {
          queryParams.append(key, String(value));
        }
      });
    }

    const queryString = queryParams.toString();
    const endpoint = `/api/tasks/tasks${queryString ? `?${queryString}` : ''}`;

    const response = await fetch(`${API_BASE_URL}${endpoint}`, {
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      if (response.status === 401) {
        authApi.setAuthToken(null); // Clear invalid token
      }
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    const result = await response.json();
    // Extract tasks from the response.data.tasks structure
    const apiTasks = Array.isArray(result) ? result : result.data?.tasks || [];

    // Transform API tasks to UI tasks
    return apiTasks.map((apiTask: ApiTask) => apiTaskToUiTask(apiTask));
  },

  // Create a new task
  createTask: async (taskData: Omit<UiTask, 'id' | 'createdAt' | 'updatedAt' | 'userId'>) => {
    const token = authApi.getAuthToken();
    if (!token) {
      throw new Error('User not authenticated');
    }

    // Transform UI task to API task format
    const apiTaskData = uiTaskToApiTask(taskData);

    const response = await fetch(`${API_BASE_URL}/api/tasks/tasks/`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(apiTaskData),
    });

    if (!response.ok) {
      if (response.status === 401) {
        authApi.setAuthToken(null); // Clear invalid token
      }
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    const result = await response.json();
    const apiTask = result.data?.task || result;

    // Transform API task back to UI task format
    return apiTaskToUiTask(apiTask);
  },

  // Get a specific task
  getTask: async (taskId: string) => {
    const token = authApi.getAuthToken();
    if (!token) {
      throw new Error('User not authenticated');
    }

    const response = await fetch(`${API_BASE_URL}/api/tasks/tasks/${taskId}`, {
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      if (response.status === 401) {
        authApi.setAuthToken(null); // Clear invalid token
      }
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    const result = await response.json();
    const apiTask = result.data?.task || result;

    // Transform API task to UI task
    return apiTaskToUiTask(apiTask as ApiTask);
  },

  // Update a task
  updateTask: async (taskId: string, taskData: Partial<UiTask>) => {
    const token = authApi.getAuthToken();
    if (!token) {
      throw new Error('User not authenticated');
    }

    // Transform UI task update to API task format
    const apiTaskData: Partial<ApiTask> = {};
    if (taskData.title !== undefined) apiTaskData.title = taskData.title;
    if (taskData.description !== undefined) apiTaskData.description = taskData.description;
    if (taskData.priority !== undefined) apiTaskData.priority = taskData.priority.toLowerCase() as 'low' | 'medium' | 'high';
    if (taskData.due_date !== undefined) apiTaskData.due_date = taskData.due_date;
    if (taskData.is_completed !== undefined) {
      apiTaskData.status = taskData.is_completed ? 'completed' : 'pending';
    }

    const response = await fetch(`${API_BASE_URL}/api/tasks/tasks/${taskId}`, {
      method: 'PUT',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(apiTaskData),
    });

    if (!response.ok) {
      if (response.status === 401) {
        authApi.setAuthToken(null); // Clear invalid token
      }
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    const result = await response.json();
    const apiTask = result.data?.task || result;

    // Transform API task back to UI task format
    return apiTaskToUiTask(apiTask);
  },

  // Delete a task
  deleteTask: async (taskId: string) => {
    const token = authApi.getAuthToken();
    if (!token) {
      throw new Error('User not authenticated');
    }

    const response = await fetch(`${API_BASE_URL}/api/tasks/tasks/${taskId}`, {
      method: 'DELETE',
      headers: {
        'Authorization': `Bearer ${token}`,
      },
    });

    if (!response.ok) {
      if (response.status === 401) {
        authApi.setAuthToken(null); // Clear invalid token
      }
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    return response.json();
  },

  // Update task completion status
  updateTaskCompletion: async (taskId: string, completed: boolean) => {
    const token = authApi.getAuthToken();
    if (!token) {
      throw new Error('User not authenticated');
    }

    const response = await fetch(`${API_BASE_URL}/api/tasks/tasks/${taskId}/complete`, {
      method: 'PATCH',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ completed }),
    });

    if (!response.ok) {
      if (response.status === 401) {
        authApi.setAuthToken(null); // Clear invalid token
      }
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    const result = await response.json();
    const apiTask = result.data?.task || result;

    // Transform API task to UI task
    return apiTaskToUiTask(apiTask as ApiTask);
  },
};