import { authApi } from "./backend-auth-api";

// Define the user type for type safety
interface User {
  id: string;
  email: string;
  name?: string;
  image?: string;
  emailVerified?: boolean;
}

// Mock signIn, signUp, and signOut functions to use our backend auth API
export const signIn = {
  email: async (credentials: { email: string; password: string }) => {
    try {
      const result = await authApi.login(credentials);
      return { data: result, error: null };
    } catch (error) {
      return { data: null, error: error instanceof Error ? error.message : 'Login failed' };
    }
  }
};

export const signUp = {
  email: async (userData: { email: string; password: string; name: string }) => {
    try {
      const result = await authApi.register(userData);
      return { data: result, error: null };
    } catch (error) {
      return { data: null, error: error instanceof Error ? error.message : 'Registration failed' };
    }
  }
};

export const signOut = async () => {
  try {
    await authApi.logout();
    return { error: null };
  } catch (error) {
    return { error: error instanceof Error ? error.message : 'Logout failed' };
  }
};

// Function to get current user/session using our backend auth API
export const getCurrentUser = async (): Promise<User | null> => {
  try {
    // Use our backend auth API to get current user
    const user = await authApi.getCurrentUser();

    // Map the backend user response to the expected format
    return {
      id: user.id,
      email: user.email,
      name: user.name,
      emailVerified: true, // Assume verified since we're using our own auth system
    };
  } catch (error) {
    // Log the error for debugging but don't crash
    console.error("Error getting current user:", error);
    return null;
  }
};

// Mock authClient object for compatibility
const authClient = {
  session: getCurrentUser,
  signIn,
  signUp,
  signOut
};

export default authClient;