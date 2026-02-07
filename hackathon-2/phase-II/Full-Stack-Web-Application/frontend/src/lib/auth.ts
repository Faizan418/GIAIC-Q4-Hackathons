/**
 * Helper functions for authentication
 */

/**
 * Get the authentication token from Better Auth
 * This implementation uses Better Auth's built-in mechanisms
 */
export async function getAuthCookie(): Promise<string | null> {
  // Better Auth handles token management automatically
  // This is a simplified implementation - in practice, Better Auth
  // manages the tokens internally
  if (typeof window !== 'undefined') {
    // In a real implementation, you might interact with Better Auth's
    // internal token storage, but typically you'd just use their hooks
    try {
      // We'll use a cookie approach that matches what Better Auth typically uses
      const cookies = document.cookie.split(';');
      for (let cookie of cookies) {
        const [name, value] = cookie.trim().split('=');
        if (name === 'better-auth-session-token') {
          return value;
        }
      }
    } catch (error) {
      console.warn('Could not access cookies:', error);
    }
  }
  return null;
}

/**
 * Check if user is authenticated using Better Auth
 */
export async function isAuthenticated(): Promise<boolean> {
  // In a real implementation, you'd check with Better Auth
  // For now, return true if we have a token
  const token = await getAuthCookie();
  return !!token;
}