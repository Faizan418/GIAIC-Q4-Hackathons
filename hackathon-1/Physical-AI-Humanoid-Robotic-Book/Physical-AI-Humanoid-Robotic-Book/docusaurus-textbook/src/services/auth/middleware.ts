import { auth } from './better-auth-client';

// This middleware is for API routes and works with the request/response pattern
// In Docusaurus, API routes are handled differently than in Next.js
// This is a server-side function that can be used in API routes

// Utility function to check if user has access to a specific resource
export const checkUserAccess = (session: any, requiredPermissions?: string[]) => {
  if (!session) {
    return false;
  }

  // Add any additional permission checks here
  // For now, just check if user is authenticated
  return true;
};

// Function to get session from request
export const getSessionFromRequest = async (req: any) => {
  try {
    // Get session from request using Better Auth
    const response = await fetch(`${process.env.BASE_URL || 'http://localhost:3000'}/api/auth/session`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
        ...req.headers
      }
    });

    if (response.ok) {
      const sessionData = await response.json();
      return sessionData;
    }
    return null;
  } catch (error) {
    console.error('Error getting session:', error);
    return null;
  }
};

// Function to protect API routes
export const protectApiRoute = async (req: any, res: any) => {
  try {
    const response = await fetch(`${process.env.BASE_URL || 'http://localhost:3000'}/api/auth/session`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
        ...req.headers
      }
    });

    if (response.ok) {
      const sessionData = await response.json();
      return sessionData;
    }

    res.status(401).json({
      authenticated: false,
      error: 'Unauthorized',
      message: 'Please sign in to access this resource'
    });
    return null;
  } catch (error) {
    console.error('Auth protection error:', error);
    res.status(401).json({
      authenticated: false,
      error: 'Unauthorized'
    });
    return null;
  }
};
