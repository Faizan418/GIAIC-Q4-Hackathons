import { auth } from '../../src/services/auth/better-auth-client';
import { validateInput, validatePassword, sanitizeInput } from '../../src/utils/auth-validation';

export default async function handler(req: any, res: any) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { email, password, softwareBackground, hardwareExperience, learningTrack, skillLevel } = req.body;

    // Validate required fields
    const validation = validateInput({ email, password }, ['email', 'password']);
    if (!validation.isValid) {
      return res.status(400).json({
        error: 'Validation failed',
        details: validation.errors
      });
    }

    // Validate password strength
    const passwordValidation = validatePassword(password);
    if (!passwordValidation.isValid) {
      return res.status(400).json({
        error: 'Password validation failed',
        details: passwordValidation.errors
      });
    }

    // Sanitize inputs
    const sanitizedSoftwareBackground = softwareBackground ? sanitizeInput(softwareBackground) : '';
    const sanitizedHardwareExperience = hardwareExperience ? sanitizeInput(hardwareExperience) : '';

    // Create user with Better Auth
    const response = await fetch(`${process.env.BASE_URL || 'http://localhost:3000'}/api/auth/sign-up`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email,
        password,
        // Store background information as metadata
        softwareBackground: sanitizedSoftwareBackground,
        hardwareExperience: sanitizedHardwareExperience,
        learningTrack: learningTrack || null,
        skillLevel: skillLevel || null
      })
    });

    const user = await response.json();

    if (!response.ok) {
      return res.status(400).json({ error: user.message || 'Signup failed' });
    }

    // Return the created user and session
    res.status(200).json({
      user: user.user,
      session: user.session
    });
  } catch (error: any) {
    console.error('Signup error:', error);
    res.status(500).json({
      error: error.message || 'Internal server error'
    });
  }
}
