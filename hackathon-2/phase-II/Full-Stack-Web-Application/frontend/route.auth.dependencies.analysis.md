# Current Route Structure and Auth Dependencies Analysis

**Date**: 2026-01-16
**Feature**: TASKAPP UI Redesign & Public Routing (1-ui-redesign-routing)

## Current Routes Identified

### Public Routes:
- `/` - Landing/home page
- `/signin` - Sign in page
- `/signup` - Sign up page

### Auth-Protected Routes:
- `/dashboard` - Dashboard page (currently uses auth via `authApi`)
- `/profile` - Profile page (likely requires auth)

### API Routes:
- `/api/auth/[...auth]` - Better Auth API routes

## Auth Dependencies Found

### 1. Signin Page (`/signin`)
- Uses `authApi.login()` to authenticate users
- Redirects to `/dashboard` after successful login
- Uses `router.refresh()` to update UI based on auth state

### 2. Signup Page (`/signup`)
- Likely follows similar auth flow to signin

### 3. Dashboard Page (`/dashboard`)
- Currently uses client-side store (`useTaskStore`) which may connect to backend API
- The store initialization calls `initializeStore()` which likely connects to backend
- Has a mix of client-side state management and backend API calls

### 4. Landing Page (`/`)
- Contains links to `/signin`, `/signup`, and `/dashboard`
- Has "Sign Up" and "Sign In" buttons in navigation

### 5. API Integration
- Uses `authApi` from `@/lib/backend-auth-api` for authentication
- Task store likely connects to backend API for data

## Implementation Plan for Public Access

### Phase 1: Remove Auth Dependencies
1. Remove auth checks from page components
2. Replace backend auth API calls with local state management
3. Update navigation to remove auth-dependent links
4. Modify task store to use local storage instead of backend API

### Phase 2: Update UI Components
1. Remove sign in/up links from public pages
2. Implement local task management
3. Apply dark-first theme
4. Ensure all pages work without authentication

### Key Files to Modify
- `frontend/app/page.tsx` - Remove auth links
- `frontend/app/dashboard/page.tsx` - Update to use local storage
- `frontend/app/signin/page.tsx` - Potentially redirect or remove
- `frontend/app/signup/page.tsx` - Potentially redirect or remove
- `frontend/lib/store.ts` - Update to use local storage
- `frontend/lib/backend-auth-api.ts` - Remove or replace auth calls