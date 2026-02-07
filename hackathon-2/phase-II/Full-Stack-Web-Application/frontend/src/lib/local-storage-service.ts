// frontend/lib/local-storage-service.ts

export interface LocalTask {
  id: string;
  title: string;
  description?: string;
  completed: boolean;
  priority: 'low' | 'medium' | 'high';
  tags: string[];
  dueDate?: string; // ISO string
  recurrence: { type: 'none' | 'daily' | 'weekly' | 'monthly'; interval: number; endDate?: string; occurrences?: number };
  createdAt: string; // ISO string
  updatedAt: string; // ISO string
  userId: string; // For public access version, we'll use a default user ID
}

export interface LocalUserPreferences {
  theme: 'light' | 'dark' | 'system';
  notifications: {
    enabled: boolean;
    dueTimeHours: number;
    deadlineReminders: boolean;
  };
  dashboard: {
    defaultView: 'list' | 'grid';
    showCompleted: boolean;
    defaultSort: 'dueDate' | 'priority' | 'title' | 'created';
    defaultSortOrder: 'asc' | 'desc';
  };
  createdAt: string;
  updatedAt: string;
}

export interface LocalWorkspace {
  tasks: LocalTask[];
  preferences: LocalUserPreferences;
}

const WORKSPACE_KEY = 'taskapp-workspace';
const DEFAULT_USER_ID = 'public-user'; // Default user ID for public access version

/**
 * Get the local workspace from storage
 */
export function getLocalWorkspace(): LocalWorkspace {
  if (typeof window === 'undefined') {
    return getDefaultWorkspace();
  }

  try {
    const stored = localStorage.getItem(WORKSPACE_KEY);
    if (stored) {
      const parsed = JSON.parse(stored);
      return {
        tasks: parsed.tasks || [],
        preferences: parsed.preferences || getDefaultPreferences()
      };
    }
  } catch (error) {
    console.error('Error reading workspace from localStorage:', error);
  }

  return getDefaultWorkspace();
}

/**
 * Save the local workspace to storage
 */
export function saveLocalWorkspace(workspace: LocalWorkspace): void {
  if (typeof window === 'undefined') {
    return;
  }

  try {
    const dataToStore = {
      tasks: workspace.tasks,
      preferences: {
        ...workspace.preferences,
        updatedAt: new Date().toISOString()
      }
    };

    localStorage.setItem(WORKSPACE_KEY, JSON.stringify(dataToStore));
  } catch (error) {
    console.error('Error saving workspace to localStorage:', error);
  }
}

/**
 * Add a new task to the local workspace
 */
export function addLocalTask(task: Omit<LocalTask, 'id' | 'createdAt' | 'updatedAt' | 'userId'>): LocalTask {
  const workspace = getLocalWorkspace();
  const newTask: LocalTask = {
    ...task,
    userId: DEFAULT_USER_ID, // Assign default user ID for public access
    id: generateId(),
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString()
  };

  workspace.tasks.push(newTask);
  saveLocalWorkspace(workspace);

  return newTask;
}

/**
 * Update an existing task in the local workspace
 */
export function updateLocalTask(id: string, updates: Partial<Omit<LocalTask, 'id' | 'createdAt'>>): LocalTask | null {
  const workspace = getLocalWorkspace();
  const taskIndex = workspace.tasks.findIndex(task => task.id === id);

  if (taskIndex === -1) {
    return null;
  }

  const updatedTask = {
    ...workspace.tasks[taskIndex],
    ...updates,
    updatedAt: new Date().toISOString()
  };

  workspace.tasks[taskIndex] = updatedTask;
  saveLocalWorkspace(workspace);

  return updatedTask;
}

/**
 * Delete a task from the local workspace
 */
export function deleteLocalTask(id: string): boolean {
  const workspace = getLocalWorkspace();
  const initialLength = workspace.tasks.length;
  workspace.tasks = workspace.tasks.filter(task => task.id !== id);

  if (workspace.tasks.length !== initialLength) {
    saveLocalWorkspace(workspace);
    return true;
  }

  return false;
}

/**
 * Get all tasks from the local workspace
 */
export function getAllLocalTasks(): LocalTask[] {
  const workspace = getLocalWorkspace();
  return workspace.tasks;
}

/**
 * Get a specific task by ID from the local workspace
 */
export function getLocalTaskById(id: string): LocalTask | null {
  const workspace = getLocalWorkspace();
  return workspace.tasks.find(task => task.id === id) || null;
}

/**
 * Update user preferences in the local workspace
 */
export function updateLocalPreferences(updates: Partial<LocalUserPreferences>): LocalUserPreferences {
  const workspace = getLocalWorkspace();
  workspace.preferences = {
    ...workspace.preferences,
    ...updates,
    updatedAt: new Date().toISOString()
  };

  saveLocalWorkspace(workspace);
  return workspace.preferences;
}

/**
 * Get user preferences from the local workspace
 */
export function getLocalPreferences(): LocalUserPreferences {
  const workspace = getLocalWorkspace();
  return workspace.preferences;
}

/**
 * Generate a unique ID
 */
function generateId(): string {
  return Date.now().toString(36) + Math.random().toString(36).substr(2, 5);
}

/**
 * Get default workspace
 */
function getDefaultWorkspace(): LocalWorkspace {
  return {
    tasks: [],
    preferences: getDefaultPreferences()
  };
}

/**
 * Get default preferences
 */
function getDefaultPreferences(): LocalUserPreferences {
  return {
    theme: 'system',
    notifications: {
      enabled: true,
      dueTimeHours: 24,
      deadlineReminders: true
    },
    dashboard: {
      defaultView: 'list',
      showCompleted: true,
      defaultSort: 'dueDate',
      defaultSortOrder: 'asc'
    },
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString()
  };
}

/**
 * Clear all local workspace data
 */
export function clearLocalWorkspace(): void {
  if (typeof window === 'undefined') {
    return;
  }

  try {
    localStorage.removeItem(WORKSPACE_KEY);
  } catch (error) {
    console.error('Error clearing workspace from localStorage:', error);
  }
}