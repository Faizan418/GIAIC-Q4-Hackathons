import { auth } from '../../src/services/auth/better-auth-client';
import { validateInput } from '../../src/utils/auth-validation';

export default async function handler(req: any, res: any) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { email, password } = req.body;

    // Validate required fields
    const validation = validateInput({ email, password }, ['email', 'password']);
    if (!validation.isValid) {
      return res.status(400).json({
        error: 'Validation failed',
        details: validation.errors
      });
    }

    // Sign in user with Better Auth
    const response = await fetch(`${process.env.BASE_URL || 'http://localhost:3000'}/api/auth/sign-in`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email,
        password,
        // This will create a session if credentials are valid
      })
    });

    const result = await response.json();

    if (!response.ok) {
      return res.status(401).json({
        error: result.message || 'Signin failed'
      });
    }

    // Get user session to return user data as well
    const sessionResponse = await fetch(`${process.env.BASE_URL || 'http://localhost:3000'}/api/auth/session`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      }
    });

    if (sessionResponse.ok) {
      const sessionData = await sessionResponse.json();
      // Return both session and user data
      res.status(200).json({
        user: sessionData.user,
        session: sessionData.session
      });
    } else {
      // If session fetch fails, return what we have
      res.status(200).json({
        session: result.session
      });
    }
  } catch (error: any) {
    console.error('Signin error:', error);
    res.status(500).json({
      error: error.message || 'Internal server error'
    });
  }
}
