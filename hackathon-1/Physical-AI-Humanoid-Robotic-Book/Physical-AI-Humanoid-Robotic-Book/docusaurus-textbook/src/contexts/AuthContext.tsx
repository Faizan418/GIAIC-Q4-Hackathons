import React, { createContext, useContext, useReducer, ReactNode, useEffect } from 'react';
import { UserPublic, UserProfilePublic } from '../types/chat';
import { useAuthApi } from '../services/auth-api';

// Define the auth state type
interface AuthState {
  user: UserPublic | null;
  profile: UserProfilePublic | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
}

// Define the auth actions type
type AuthAction =
  | { type: 'LOGIN_START' }
  | { type: 'LOGIN_SUCCESS'; payload: { user: UserPublic; profile?: UserProfilePublic } }
  | { type: 'LOGIN_FAILURE'; payload: string }
  | { type: 'REGISTER_START' }
  | { type: 'REGISTER_SUCCESS'; payload: { user: UserPublic; profile?: UserProfilePublic } }
  | { type: 'REGISTER_FAILURE'; payload: string }
  | { type: 'LOGOUT' }
  | { type: 'SET_ERROR'; payload: string | null }
  | { type: 'LOAD_USER_START' }
  | { type: 'LOAD_USER_SUCCESS'; payload: { user: UserPublic; profile?: UserProfilePublic } }
  | { type: 'LOAD_USER_FAILURE'; payload: string }
  | { type: 'UPDATE_PROFILE'; payload: UserProfilePublic };

// Initial state
const initialState: AuthState = {
  user: null,
  profile: null,
  isAuthenticated: false,
  isLoading: false,
  error: null,
};

// Create the auth context
const AuthContext = createContext<{
  state: AuthState;
  login: (email: string, password: string) => Promise<void>;
  register: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  loadUser: () => Promise<void>;
  updateProfile: (profileData: UserProfilePublic) => void;
  clearError: () => void;
} | undefined>(undefined);

// Reducer function
const authReducer = (state: AuthState, action: AuthAction): AuthState => {
  switch (action.type) {
    case 'LOGIN_START':
    case 'REGISTER_START':
    case 'LOAD_USER_START':
      return {
        ...state,
        isLoading: true,
        error: null,
      };
    case 'LOGIN_SUCCESS':
    case 'REGISTER_SUCCESS':
    case 'LOAD_USER_SUCCESS':
      return {
        ...state,
        user: action.payload.user,
        profile: action.payload.profile || state.profile,
        isAuthenticated: true,
        isLoading: false,
        error: null,
      };
    case 'LOGIN_FAILURE':
    case 'REGISTER_FAILURE':
    case 'LOAD_USER_FAILURE':
      return {
        ...state,
        user: null,
        profile: null,
        isAuthenticated: false,
        isLoading: false,
        error: action.payload,
      };
    case 'LOGOUT':
      return {
        ...state,
        user: null,
        profile: null,
        isAuthenticated: false,
        isLoading: false,
        error: null,
      };
    case 'SET_ERROR':
      return {
        ...state,
        error: action.payload,
      };
    case 'UPDATE_PROFILE':
      return {
        ...state,
        profile: action.payload,
      };
    default:
      return state;
  }
};

// Auth provider component
export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);
  const authApi = useAuthApi();

  // Load user on initial render if token exists
  useEffect(() => {
    const loadUserFromStorage = async () => {
      if (authApi.isAuthenticated()) {
        await loadUser();
      }
    };
    loadUserFromStorage();
  }, []);

  // Login function
  const login = async (email: string, password: string) => {
    try {
      dispatch({ type: 'LOGIN_START' });
      const response = await authApi.login(email, password);

      dispatch({
        type: 'LOGIN_SUCCESS',
        payload: { user: response.user },
      });
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Login failed';
      dispatch({ type: 'LOGIN_FAILURE', payload: errorMessage });
      throw error;
    }
  };

  // Register function
  const register = async (email: string, password: string) => {
    try {
      dispatch({ type: 'REGISTER_START' });
      const response = await authApi.register({ email, password });

      dispatch({
        type: 'REGISTER_SUCCESS',
        payload: { user: response.user },
      });
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Registration failed';
      dispatch({ type: 'REGISTER_FAILURE', payload: errorMessage });
      throw error;
    }
  };

  // Logout function
  const logout = async () => {
    try {
      await authApi.logout();
      dispatch({ type: 'LOGOUT' });
    } catch (error) {
      console.error('Logout error:', error);
      dispatch({ type: 'LOGOUT' });
    }
  };

  // Load user function
  const loadUser = async () => {
    try {
      dispatch({ type: 'LOAD_USER_START' });
      const user = await authApi.getCurrentUser();

      // Try to get the user profile as well
      let profile: UserProfilePublic | undefined;
      try {
        profile = await authApi.getUserProfile();
      } catch (error) {
        // Profile might not exist yet, which is fine
        console.log('No profile found for user, this is expected for new users');
      }

      dispatch({
        type: 'LOAD_USER_SUCCESS',
        payload: { user, profile },
      });
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Failed to load user';
      dispatch({ type: 'LOAD_USER_FAILURE', payload: errorMessage });
    }
  };

  // Update profile function
  const updateProfile = (profileData: UserProfilePublic) => {
    dispatch({
      type: 'UPDATE_PROFILE',
      payload: profileData,
    });
  };

  // Clear error function
  const clearError = () => {
    dispatch({ type: 'SET_ERROR', payload: null });
  };

  return (
    <AuthContext.Provider
      value={{
        state,
        login,
        register,
        logout,
        loadUser,
        updateProfile,
        clearError,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
