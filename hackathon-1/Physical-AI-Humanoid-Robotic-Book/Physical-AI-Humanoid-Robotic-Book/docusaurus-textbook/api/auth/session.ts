import { auth } from '../../src/services/auth/better-auth-client';

export default async function handler(req: any, res: any) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Get current session
    const response = await fetch(`${process.env.BASE_URL || 'http://localhost:3000'}/api/auth/session`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
        ...req.headers
      }
    });

    if (response.ok) {
      const sessionData = await response.json();
      // Return user session data if authenticated
      res.status(200).json({
        authenticated: true,
        user: sessionData.user,
        session: sessionData.session
      });
    } else {
      // Return unauthenticated response
      res.status(200).json({
        authenticated: false,
        user: null
      });
    }
  } catch (error: any) {
    console.error('Session check error:', error);
    res.status(500).json({
      authenticated: false,
      error: error.message || 'Internal server error'
    });
  }
}
