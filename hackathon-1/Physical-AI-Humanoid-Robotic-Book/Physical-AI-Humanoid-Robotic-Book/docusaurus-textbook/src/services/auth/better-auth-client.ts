import { betterAuth } from 'better-auth';

// Initialize Better Auth with cookie-based sessions
export const auth = betterAuth({
  database: {
    provider: 'sqlite',
    url: process.env.DATABASE_URL || './db.sqlite',
  },
  secret: process.env.AUTH_SECRET || 'your-secret-key-change-in-production',
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  socialProviders: {},
  session: {
    expiresIn: 7 * 24 * 60 * 60 * 1000, // 7 days
    cookie: {
      secure: process.env.NODE_ENV === 'production',
      sameSite: 'lax',
      path: '/',
    },
  },
  user: {
    // Add custom fields for user metadata
    additionalFields: {
      softwareBackground: {
        type: 'string',
        required: false,
      },
      hardwareExperience: {
        type: 'string',
        required: false,
      },
      learningTrack: {
        type: 'string',
        required: false,
      },
      skillLevel: {
        type: 'string',
        required: false,
      },
    },
  },
});

// Export the auth client instance
export default auth;
