# Research: Auth & Onboarding System

## Decision: Better Auth Integration Approach
**Rationale**: Better Auth needs to be properly integrated with Next.js App Router to handle authentication flows, session management, and user data persistence. The integration must follow Next.js conventions while maintaining security best practices.
**Alternatives considered**:
- NextAuth.js instead of Better Auth
- Custom authentication implementation
- Third-party authentication services (Auth0, Firebase Auth)
**Chosen approach**: Better Auth as specified in the constitution and feature requirements, with proper Next.js App Router integration using middleware for protected routes.

## Decision: Profile Data Storage Strategy
**Rationale**: Profile data (software/hardware background, learning track) needs to be stored separately from authentication data as specified in the constitution. This requires a separate database schema and integration with the authentication system.
**Alternatives considered**:
- Store profile data in authentication tokens
- Use a single database table for both auth and profile data
- Client-side storage with server synchronization
**Chosen approach**: Separate database table for profile data linked to user ID from Better Auth, with server-side validation and persistence.

## Decision: Onboarding Flow Implementation
**Rationale**: The onboarding flow must be triggered for users with incomplete profiles, and must collect all required background information before allowing access to personalized content. This requires middleware to check profile completeness.
**Alternatives considered**:
- Optional onboarding with incentives to complete
- Progressive onboarding during content consumption
- Modal-based onboarding instead of dedicated page
**Chosen approach**: Mandatory redirect to onboarding page for users with incomplete profiles, with all required fields collected upfront.

## Decision: Personalization Logic Implementation
**Rationale**: Content personalization must be implemented according to the specified rules (BEGINNER → foundations first, HARDWARE_ONLY → hide AI/ML, FULL_ROBOTICS → unlock ROS/control systems). This requires a flexible system that can adapt content presentation based on profile data.
**Alternatives considered**:
- Client-side personalization only
- Static content with client-side filtering
- Separate content versions for each track
**Chosen approach**: Server-side personalization with dynamic content loading based on user profile, with fallbacks for users without profiles.

## Decision: CSS Animation Implementation
**Rationale**: The constitution specifies CSS-only animations with hardware-accelerated transforms, no third-party libraries, and specific timing constraints. This requires careful implementation of animation tokens and consistent styling.
**Alternatives considered**:
- Using a CSS animation library like Framer Motion
- JavaScript-based animations
- GIFs or video-based animations
**Chosen approach**: Pure CSS animations using keyframes, transforms, and transitions following the specified animation tokens.

## Decision: Workflow Implementation Strategy
**Rationale**: The CI/CD workflows must include auth-check.yml to verify constitution compliance, with proper validation of auth directories and configuration. This requires specific GitHub Actions implementations.
**Alternatives considered**:
- Single workflow for all checks
- External validation tools
- Manual compliance checks
**Chosen approach**: Three separate workflows (ci.yml, auth-check.yml, deploy.yml) with specific validation steps for each.

## Decision: Session Management and Middleware
**Rationale**: Proper session management is needed to track authentication state and profile completeness across page navigations. Next.js middleware is required to protect routes and enforce onboarding flow.
**Alternatives considered**:
- Client-side session storage only
- Server-side sessions with cookies
- JWT tokens stored client-side
**Chosen approach**: Better Auth's built-in session management with Next.js middleware for route protection and profile completeness checks.
