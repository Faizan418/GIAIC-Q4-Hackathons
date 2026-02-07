# Data Model: Auth-Aware Navbar Dropdown

## Entities

### Authentication State
**Description**: Represents the current authentication status of the user
- **States**: `loading`, `authenticated`, `unauthenticated`
- **Source**: Better Auth session context
- **Validation**: Must be one of the three defined states

### Navbar Item
**Description**: Represents the custom authentication-aware dropdown component
- **Properties**:
  - `type`: 'authDropdown' (for Docusaurus navbar configuration)
  - `position`: 'right' (navbar placement)
- **Validation**: Type must match component file name for Docusaurus resolution

### Better Auth Session
**Description**: Represents the user session data from Better Auth
- **Properties**:
  - `user`: User object when authenticated (null when unauthenticated)
  - `isLoading`: Boolean indicating session loading state
  - `isAuthenticated`: Boolean indicating authentication status
- **Validation**: Follows Better Auth API contract

## State Transitions

### Authentication State Transitions
- `loading` → `authenticated`: When session loads and user is logged in
- `loading` → `unauthenticated`: When session loads and user is not logged in
- `authenticated` → `unauthenticated`: When user logs out
- `unauthenticated` → `authenticated`: When user logs in

### Navbar Label Transitions
- When state is `loading`: Component does not render
- When state is `unauthenticated`: Label shows "Sign In"
- When state is `authenticated`: Label shows "Dashboard"
