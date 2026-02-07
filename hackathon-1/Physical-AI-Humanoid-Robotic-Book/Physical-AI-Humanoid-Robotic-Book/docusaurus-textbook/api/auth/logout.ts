import { auth } from '../../src/services/auth/better-auth-client';

export default async function handler(req: any, res: any) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Sign out user with Better Auth
    const response = await fetch(`${process.env.BASE_URL || 'http://localhost:3000'}/api/auth/sign-out`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...req.headers
      }
    });

    const result = await response.json();

    if (!response.ok) {
      return res.status(400).json({
        error: result.message || 'Logout failed'
      });
    }

    // Return success response
    res.status(200).json({
      message: 'Successfully logged out'
    });
  } catch (error: any) {
    console.error('Logout error:', error);
    res.status(500).json({
      error: error.message || 'Internal server error'
    });
  }
}
