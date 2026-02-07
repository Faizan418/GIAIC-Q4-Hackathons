// API service for authentication functionality
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { UserPublic, UserProfilePublic, AuthResponse } from '../types/chat';

interface UserCreate {
  email: string;
  password: string;
}

interface UserProfileCreate {
  software_level: string;
  known_languages: string[];
  hardware_experience: string;
  learning_track: string;
  boards_used?: string[];
}

interface UserProfileUpdate {
  software_level?: string;
  known_languages?: string[];
  hardware_experience?: string;
  learning_track?: string;
  boards_used?: string[];
}

export function useAuthApi() {
  const { siteConfig } = useDocusaurusContext();

  const API_BASE_URL = siteConfig.customFields
    ?.CHAT_API_URL as string;

  if (!API_BASE_URL) {
    throw new Error("CHAT_API_URL is not defined");
  }

  // Get the token from localStorage
  const getToken = (): string | null => {
    if (typeof window !== 'undefined') {
      return localStorage.getItem('auth_token');
    }
    return null;
  };

  // Set the token in localStorage
  const setToken = (token: string): void => {
    if (typeof window !== 'undefined') {
      localStorage.setItem('auth_token', token);
    }
  };

  // Remove the token from localStorage
  const removeToken = (): void => {
    if (typeof window !== 'undefined') {
      localStorage.removeItem('auth_token');
    }
  };

  // Get authorization headers
  const getAuthHeaders = () => {
    const token = getToken();
    return {
      'Content-Type': 'application/json',
      ...(token ? { 'Authorization': `Bearer ${token}` } : {}),
    };
  };

  return {
    // Registration
    async register(userData: UserCreate): Promise<AuthResponse> {
      try {
        const response = await fetch(`${API_BASE_URL}/auth/register`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(userData),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || `API request failed with status ${response.status}`);
        }

        const data = await response.json();

        // Store the token in localStorage
        if (data.access_token) {
          setToken(data.access_token);
        }

        return data;
      } catch (error) {
        console.error('Error in register API call:', error);
        throw error;
      }
    },

    // Login
    async login(email: string, password: string): Promise<AuthResponse> {
      try {
        // Create form data for the login request
        const formData = new URLSearchParams();
        formData.append('username', email);
        formData.append('password', password);

        const response = await fetch(`${API_BASE_URL}/auth/token`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
          },
          body: formData,
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || `API request failed with status ${response.status}`);
        }

        const data = await response.json();

        // Store the token in localStorage
        if (data.access_token) {
          setToken(data.access_token);
        }

        // Get the user info to return with the token
        const userResponse = await fetch(`${API_BASE_URL}/auth/me`, {
          headers: getAuthHeaders(),
        });

        if (!userResponse.ok) {
          throw new Error(`Failed to get user info: ${userResponse.status}`);
        }

        const user = await userResponse.json();

        return {
          user,
          access_token: data.access_token,
          token_type: data.token_type
        };
      } catch (error) {
        console.error('Error in login API call:', error);
        throw error;
      }
    },

    // Logout
    async logout(): Promise<void> {
      // Just remove the token from localStorage
      removeToken();
    },

    // Get current user
    async getCurrentUser(): Promise<UserPublic> {
      try {
        const response = await fetch(`${API_BASE_URL}/auth/me`, {
          headers: getAuthHeaders(),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || `API request failed with status ${response.status}`);
        }

        const data = await response.json();
        return data;
      } catch (error) {
        console.error('Error in getCurrentUser API call:', error);
        throw error;
      }
    },

    // Get user profile
    async getUserProfile(): Promise<UserProfilePublic> {
      try {
        const response = await fetch(`${API_BASE_URL}/auth/profile`, {
          headers: getAuthHeaders(),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || `API request failed with status ${response.status}`);
        }

        const data = await response.json();
        return data;
      } catch (error) {
        console.error('Error in getUserProfile API call:', error);
        throw error;
      }
    },

    // Create or update user profile
    async createOrUpdateProfile(profileData: UserProfileCreate): Promise<UserProfilePublic> {
      try {
        const response = await fetch(`${API_BASE_URL}/auth/profile`, {
          method: 'POST',
          headers: getAuthHeaders(),
          body: JSON.stringify(profileData),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || `API request failed with status ${response.status}`);
        }

        const data = await response.json();
        return data;
      } catch (error) {
        console.error('Error in createOrUpdateProfile API call:', error);
        throw error;
      }
    },

    // Update user profile
    async updateProfile(profileData: UserProfileUpdate): Promise<UserProfilePublic> {
      try {
        const response = await fetch(`${API_BASE_URL}/auth/profile`, {
          method: 'PATCH',
          headers: getAuthHeaders(),
          body: JSON.stringify(profileData),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || `API request failed with status ${response.status}`);
        }

        const data = await response.json();
        return data;
      } catch (error) {
        console.error('Error in updateProfile API call:', error);
        throw error;
      }
    },

    // Check if profile is complete
    async isProfileComplete(): Promise<boolean> {
      try {
        const response = await fetch(`${API_BASE_URL}/auth/profile-complete`, {
          headers: getAuthHeaders(),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || `API request failed with status ${response.status}`);
        }

        const data = await response.json();
        return data.profile_complete;
      } catch (error) {
        console.error('Error in isProfileComplete API call:', error);
        throw error;
      }
    },

    // Check if user is authenticated
    isAuthenticated(): boolean {
      return !!getToken();
    },

    // Get token
    getToken(): string | null {
      return getToken();
    }
  };
}
