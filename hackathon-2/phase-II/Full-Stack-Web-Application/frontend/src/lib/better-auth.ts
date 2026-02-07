import { betterAuth } from 'better-auth';

// Initialize Better Auth server-side
export const auth = betterAuth({
  // Use in-memory database for development to avoid database adapter issues
  database: process.env.NODE_ENV === 'production'
    ? {
        provider: 'postgresql',
        url: process.env.DATABASE_URL!,
      }
    : {
        provider: 'sqlite',
        url: 'sqlite::memory:',
      }, // Use in-memory SQLite for development
  // Add your app domain
  app: {
    name: 'TASKAPP',
    baseUrl: process.env.NEXT_PUBLIC_APP_URL || 'http://localhost:3000',
  },
  // Enable email & password authentication
  emailAndPassword: {
    enabled: true,
  },
  // Configure social providers (optional)
  socialProviders: process.env.GOOGLE_CLIENT_ID && process.env.GOOGLE_CLIENT_SECRET ? {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    },
  } : {},
  // Add secret for JWT
  secret: process.env.BETTER_AUTH_SECRET || 'fallback-secret-for-development',
  // Session configuration
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
    updateAge: 24 * 60 * 60, // 24 hours
  },
  // CSRF configuration
  csrf: {
    enabled: true,
  }
});

export default auth;