# Quickstart: Auth-Aware Navbar Dropdown

## Overview
This guide will help you implement an authentication-aware navbar dropdown that integrates with Better Auth and follows Docusaurus best practices.

## Prerequisites
- Docusaurus v3 installed
- Better Auth configured in your project
- TypeScript 5.0+ environment

## Implementation Steps

### 1. Configure Navbar Item
Update your `docusaurus.config.ts` to include the auth dropdown:

```js
module.exports = {
  // ... other config
  navbar: {
    items: [
      // ... other items
      {
        type: 'authDropdown',
        position: 'right',
      },
    ],
  },
};
```

### 2. Create AuthDropdown Component
Create the component at `src/theme/NavbarItem/AuthDropdown.tsx`:

```tsx
import React from 'react';
import { useSession, signOut } from 'better-auth/react';
import { useHistory } from '@docusaurus/router';

const AuthDropdown = () => {
  const { data: session, isLoading } = useSession();

  if (isLoading) {
    return null; // Don't render during loading
  }

  if (session?.user) {
    // Authenticated state
    return (
      <div className="navbar__item dropdown dropdown--hoverable">
        <a className="navbar__link" href="/dashboard">Dashboard</a>
        <ul className="dropdown__menu">
          <li><a className="dropdown__link" href="/dashboard">Dashboard</a></li>
          <li>
            <button
              type="button"
              className="dropdown__link"
              onClick={async () => {
                await signOut();
                window.location.href = '/';
              }}
            >
              Logout
            </button>
          </li>
        </ul>
      </div>
    );
  } else {
    // Unauthenticated state
    return (
      <div className="navbar__item dropdown dropdown--hoverable">
        <a className="navbar__link">Sign In</a>
        <ul className="dropdown__menu">
          <li><a className="dropdown__link" href="/auth/signin">Sign In</a></li>
          <li><a className="dropdown__link" href="/auth/signup">Sign Up</a></li>
        </ul>
      </div>
    );
  }
};

export default AuthDropdown;
```

### 3. Verify Implementation
- Ensure the navbar renders correctly in both authenticated and unauthenticated states
- Test that dropdown opens/closes properly using Docusaurus native behavior
- Verify logout functionality works correctly
- Check that loading state is handled properly

## Key Features
- Uses Docusaurus native dropdown behavior
- Integrates with Better Auth session context
- Follows accessibility standards
- Responsive design for mobile and desktop
- Proper loading state handling
