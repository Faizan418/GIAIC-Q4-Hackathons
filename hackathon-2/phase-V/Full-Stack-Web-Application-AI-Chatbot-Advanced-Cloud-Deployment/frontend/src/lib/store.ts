// frontend/lib/store.ts
import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import { Task } from '@/types/task';
import { LocalUserPreferences } from './local-storage-service';
import {
  getAllLocalTasks,
  addLocalTask,
  updateLocalTask,
  deleteLocalTask,
  getLocalPreferences,
  updateLocalPreferences,
  type LocalTask
} from './local-storage-service';

interface TaskStore {
  tasks: Task[];
  preferences: LocalUserPreferences;
  isLoading: boolean;
  isTaskLoading: boolean; // Loading state specifically for task operations
  // Add methods for CRUD operations
  addTask: (task: Omit<Task, 'id' | 'createdAt' | 'updatedAt' | 'userId'>) => Promise<void>;
  updateTask: (id: string, updates: Partial<Task>) => Promise<void>;
  deleteTask: (id: string) => Promise<void>;
  reorderTasks: (tasks: Task[]) => void;
  setPreferences: (updates: Partial<LocalUserPreferences>) => void;
  fetchTasks: () => Promise<void>;
  initializeStore: () => Promise<void>;
}

export const useTaskStore = create(
  persist<TaskStore>(
    (set, get) => ({
      tasks: [],
      preferences: getLocalPreferences(), // Load preferences from local storage
      isLoading: false,
      isTaskLoading: false,

      fetchTasks: async () => {
        set({ isLoading: true });
        try {
          // Simulate a small delay for better UX
          await new Promise(resolve => setTimeout(resolve, 100));
          const localTasks = getAllLocalTasks();

          // Convert LocalTask to Task format
          const tasks: Task[] = localTasks.map(localTask => ({
            id: localTask.id,
            title: localTask.title,
            description: localTask.description,
            is_completed: localTask.completed,
            priority: localTask.priority.toUpperCase() as 'LOW' | 'MEDIUM' | 'HIGH',
            due_date: localTask.dueDate,
            recurrence_pattern: localTask.recurrence?.type,
            created_at: localTask.createdAt,
            updated_at: localTask.updatedAt,
            user_id: localTask.userId,
            order_index: '0' // Default value for local storage tasks
          }));

          set({ tasks, isLoading: false });
        } catch (error) {
          console.error('Failed to fetch tasks from local storage:', error);
          set({ isLoading: false });
        }
      },

      initializeStore: async () => {
        // Load tasks and preferences from local storage
        set({ isLoading: true });
        try {
          await get().fetchTasks();
          // Preferences are already loaded in the initial state
          set({ isLoading: false });
        } catch (error) {
          console.error('Failed to initialize store:', error);
          set({ isLoading: false });
        }
      },

      addTask: async (taskData) => {
        set({ isTaskLoading: true });
        try {
          // Convert Task format to LocalTask format for storage
          const taskForStorage = {
            ...taskData,
            userId: 'public-user', // Default user ID for public access version
            completed: taskData.is_completed || false,
            priority: (taskData.priority?.toLowerCase() || 'medium') as 'low' | 'medium' | 'high',
            dueDate: taskData.due_date,
            recurrence: { type: taskData.recurrence_pattern || 'none', interval: 1 },
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString(),
            tags: [], // Default empty tags array
            id: Date.now().toString() + Math.random().toString(36).substr(2, 5) // Generate temp ID
          } as Omit<LocalTask, 'id'> & { id: string };

          const newLocalTask = addLocalTask(taskForStorage);

          // Convert back to Task format
          const newTask: Task = {
            id: newLocalTask.id,
            title: newLocalTask.title,
            description: newLocalTask.description,
            is_completed: newLocalTask.completed,
            priority: newLocalTask.priority.toUpperCase() as 'LOW' | 'MEDIUM' | 'HIGH',
            due_date: newLocalTask.dueDate,
            recurrence_pattern: newLocalTask.recurrence?.type,
            created_at: newLocalTask.createdAt,
            updated_at: newLocalTask.updatedAt,
            user_id: newLocalTask.userId,
            order_index: '0' // Default value
          };

          // Simulate a small delay for better UX
          await new Promise(resolve => setTimeout(resolve, 300));

          set((state) => ({
            tasks: [...state.tasks, newTask],
            isTaskLoading: false
          }));
        } catch (error) {
          console.error('Failed to add task to local storage:', error);
          set({ isTaskLoading: false });
          throw error;
        }
      },

      updateTask: async (id, updates) => {
        set({ isTaskLoading: true });
        try {
          // Convert Task updates to LocalTask format for storage
          const updatesForStorage: Partial<Omit<LocalTask, 'id' | 'userId' | 'createdAt' | 'updatedAt'>> = {};

          if (updates.title !== undefined) updatesForStorage.title = updates.title;
          if (updates.description !== undefined) updatesForStorage.description = updates.description;
          if (updates.is_completed !== undefined) updatesForStorage.completed = updates.is_completed;
          if (updates.priority !== undefined) updatesForStorage.priority = updates.priority.toLowerCase() as 'low' | 'medium' | 'high';
          if (updates.due_date !== undefined) updatesForStorage.dueDate = updates.due_date;
          if (updates.recurrence_pattern !== undefined) updatesForStorage.recurrence = { type: updates.recurrence_pattern as 'none' | 'daily' | 'weekly' | 'monthly', interval: 1 };

          const updatedLocalTask = updateLocalTask(id, updatesForStorage);
          if (updatedLocalTask) {
            // Convert back to Task format
            const updatedTask: Task = {
              id: updatedLocalTask.id,
              title: updatedLocalTask.title,
              description: updatedLocalTask.description,
              is_completed: updatedLocalTask.completed,
              priority: updatedLocalTask.priority.toUpperCase() as 'LOW' | 'MEDIUM' | 'HIGH',
              due_date: updatedLocalTask.dueDate,
              recurrence_pattern: updatedLocalTask.recurrence?.type,
              created_at: updatedLocalTask.createdAt,
              updated_at: updatedLocalTask.updatedAt,
              user_id: updatedLocalTask.userId,
              order_index: '0' // Default value
            };

            // Simulate a small delay for better UX
            await new Promise(resolve => setTimeout(resolve, 300));

            set((state) => ({
              tasks: state.tasks.map(task =>
                task.id === id ? updatedTask : task
              ),
              isTaskLoading: false
            }));
          } else {
            set({ isTaskLoading: false });
          }
        } catch (error) {
          console.error('Failed to update task in local storage:', error);
          set({ isTaskLoading: false });
          throw error;
        }
      },

      deleteTask: async (id) => {
        set({ isTaskLoading: true });
        try {
          const success = deleteLocalTask(id);
          if (success) {
            // Simulate a small delay for better UX
            await new Promise(resolve => setTimeout(resolve, 300));

            set((state) => ({
              tasks: state.tasks.filter(task => task.id !== id),
              isTaskLoading: false
            }));
          } else {
            set({ isTaskLoading: false });
          }
        } catch (error) {
          console.error('Failed to delete task from local storage:', error);
          set({ isTaskLoading: false });
          throw error;
        }
      },

      reorderTasks: (orderedTasks) => set({ tasks: orderedTasks }),

      setPreferences: (updates) => {
        const updatedPrefs = updateLocalPreferences(updates);
        set({ preferences: updatedPrefs });
      }
    }),
    {
      name: 'taskapp-storage-v2', // Updated local storage key to avoid conflicts
      storage: createJSONStorage(() => localStorage) // Use localStorage
    }
  )
);