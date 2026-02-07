# Component Contract: AuthDropdown

## Overview
The AuthDropdown component provides an authentication-aware navbar dropdown that displays different content based on user authentication status.

## Component Interface

### Props
The component does not accept any props as it's a Docusaurus NavbarItem extension that gets configured via the navbar configuration.

### Dependencies
- `better-auth/react`: For session management and authentication functions
- `@docusaurus/router`: For navigation after logout (if needed)

## Behavior Contracts

### Authentication State Handling
- **Loading State**: When `isLoading` is true, component returns `null` (does not render)
- **Authenticated State**: Shows "Dashboard" as the dropdown label with Dashboard and Logout options
- **Unauthenticated State**: Shows "Sign In" as the dropdown label with Sign In and Sign Up options

### Session Integration
- **Input**: Uses `useSession()` hook from Better Auth to get current session state
- **Output**: Renders appropriate UI based on session data
- **Error Handling**: Defaults to unauthenticated state if session check fails

### Navigation Contracts
- **Sign In**: Navigates to `/auth/signin`
- **Sign Up**: Navigates to `/auth/signup`
- **Dashboard**: Navigates to `/dashboard`
- **Logout**: Calls `signOut()` and redirects to `/`

### Styling Contracts
- **Classes Used**:
  - `navbar__item`: Container for navbar item
  - `dropdown`: Dropdown container
  - `dropdown--hoverable`: Enables hover behavior
  - `navbar__link`: Link styling
  - `dropdown__menu`: Dropdown menu container
  - `dropdown__link`: Dropdown item styling

### Accessibility Contracts
- **Keyboard Navigation**: Supports Docusaurus default keyboard navigation
- **Button Attributes**: Logout button includes `type="button"` attribute
- **Semantic HTML**: Uses proper HTML structure (`ul > li > a/button`)

## State Transitions
- Component automatically transitions between states based on session data
- No external state management required
- Follows Docusaurus dropdown behavior for open/close

## Error Handling
- If authentication API fails, defaults to unauthenticated state
- If Better Auth is not initialized, defaults to unauthenticated state
- Logout errors are handled by Better Auth library
