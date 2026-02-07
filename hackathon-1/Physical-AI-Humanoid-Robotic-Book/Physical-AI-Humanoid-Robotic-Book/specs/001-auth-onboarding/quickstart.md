# Quickstart: Auth & Onboarding System

## Overview
This guide will help you get the authentication and onboarding system up and running quickly. The system implements Better Auth for secure authentication, collects user background information for personalization, and enforces UI animation standards.

## Prerequisites
- Node.js 18+
- Next.js 14+ with App Router
- TypeScript
- A GitHub repository for workflow implementation

## Environment Setup

1. Install Better Auth and related dependencies:
```bash
npm install better-auth @better-auth/node next
```

2. Create a `.env.local` file in your project root:
```bash
# Better Auth configuration
BETTER_AUTH_URL=http://localhost:3000
BETTER_AUTH_SECRET=your-super-secret-key-here

# Database configuration for profile storage
DATABASE_URL=your-database-connection-string
```

## Running the Auth System

1. Initialize Better Auth in `app/lib/auth.ts`:
```typescript
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  emailAndPassword: {
    enabled: true,
  },
  session: {
    cookieCache: true,
  },
});
```

2. Create authentication routes in `app/(auth)/`:
   - `app/(auth)/signin/page.tsx`
   - `app/(auth)/signup/page.tsx`

3. Start your Next.js development server:
```bash
npm run dev
```

The auth system will be available at:
- Signup: http://localhost:3000/signup
- Signin: http://localhost:3000/signin

## Using the Onboarding Flow

1. After successful registration, users are automatically redirected to the onboarding flow at `/onboarding`
2. The onboarding flow collects:
   - Software background (skill level and known languages)
   - Hardware background (experience level and boards used)
   - Learning track preference (SOFTWARE_ONLY, HARDWARE_ONLY, or FULL_ROBOTICS)
3. Profile data is stored in the application database and linked to the user account

## Personalization Features

The system automatically applies personalization rules:
- **BEGINNER**: Foundational materials are prioritized
- **HARDWARE_ONLY**: AI/ML content is hidden or de-emphasized
- **FULL_ROBOTICS**: ROS and control systems content is unlocked

## UI Animation Standards

The system implements CSS-only animations following these standards:
- Form container entrance animation (600ms ease-out)
- Button hover effects (translateY(-2px))
- Button active states (scale(0.96))
- Loading spinners with CSS animations
- All animations use hardware-accelerated transforms

## GitHub Actions Workflows

The system includes three workflow files in `.github/workflows/`:
- `ci.yml`: Build and type safety checks
- `auth-check.yml`: Constitution compliance verification
- `deploy.yml`: Production deployment (only from main branch)

## Key Features
- **Secure Authentication**: Email/password authentication via Better Auth
- **Profile Collection**: Mandatory onboarding with background information
- **Content Personalization**: Adaptive content based on user profile
- **Animated UI**: Consistent CSS animations across auth screens
- **Workflow Governance**: CI/CD enforcement with constitution checks
- **Accessibility**: Keyboard navigation and screen reader support
