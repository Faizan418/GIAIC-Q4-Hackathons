import React, { useEffect, useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useLocation } from '@docusaurus/router';

interface ProfileGuardProps {
  children: React.ReactNode;
  onboardingPath?: string;
  fallback?: React.ReactNode;
}

const ProfileGuard: React.FC<ProfileGuardProps> = ({
  children,
  onboardingPath = '/onboarding',
  fallback = <div>Checking profile...</div>
}) => {
  const { state, loadUser } = useAuth();
  const [checked, setChecked] = useState(false);
  const location = useLocation();

  useEffect(() => {
    const checkProfile = async () => {
      if (!state.isAuthenticated && !state.user) {
        // Try to load user if not already loaded
        await loadUser();
      }
      setChecked(true);
    };

    checkProfile();
  }, [state.isAuthenticated, state.user, loadUser]);

  // If not authenticated, allow to continue (auth guard should handle this)
  if (!state.isAuthenticated) {
    return <>{children}</>;
  }

  // If still checking, show fallback
  if (!checked) {
    return <>{fallback}</>;
  }

  // If profile is not complete and user is not on onboarding page, redirect
  if (!state.user?.profile_complete && location.pathname !== onboardingPath) {
    // Redirect using window.location
    if (typeof window !== 'undefined') {
      window.location.href = onboardingPath;
    }
    return <div>Redirecting...</div>;
  }

  // If profile is complete or on onboarding page, render children
  return <>{children}</>;
};

export default ProfileGuard;
