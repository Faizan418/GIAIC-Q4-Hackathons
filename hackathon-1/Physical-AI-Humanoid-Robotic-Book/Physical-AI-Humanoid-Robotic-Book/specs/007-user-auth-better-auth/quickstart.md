# Quickstart: Better Auth Integration for Docusaurus Platform

## Prerequisites

- Node.js LTS installed
- Docusaurus project set up
- Git repository initialized

## Installation

1. Install Better Auth dependency:
   ```bash
   npm install better-auth
   ```

2. Verify installation in package.json:
   ```json
   {
     "dependencies": {
       "better-auth": "^0.x.x"
     }
   }
   ```

## Configuration

1. Create API routes directory: `api/auth/`

2. Create the following API route files:
   - `api/auth/signin.ts`
   - `api/auth/signup.ts`
   - `api/auth/logout.ts`

3. Configure Better Auth in your application with cookie-based sessions.

## Frontend Setup

1. Create auth pages in `src/pages/auth/`:
   - `signin.tsx`
   - `signup.tsx`
   - `dashboard.tsx`

2. Update navbar configuration to include auth-aware behavior.

## Protected Routes

1. Implement middleware to protect:
   - `/dashboard`
   - `/docs/advanced/*`

2. Redirect unauthenticated users to `/auth/signin`.

## Testing

1. Run the Docusaurus development server:
   ```bash
   npm run start
   ```

2. Test user registration flow with background information collection.

3. Verify protected routes redirect unauthenticated users.

4. Confirm dashboard displays personalized user information.

## Deployment

1. Ensure environment variables are configured for production.

2. Verify cookie settings work in production environment.

3. Test complete auth flow on deployed site.
