import React, { useState, useEffect } from 'react';

interface ProtectedRouteProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
}

export const ProtectedRoute: React.FC<ProtectedRouteProps> = ({
  children,
  fallback = (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <div className="text--center padding-vert--md">
            <div className="loading-spinner">Checking authentication...</div>
          </div>
        </div>
      </div>
    </div>
  )
}) => {
  const [isClient, setIsClient] = useState(false);
  const [authenticated, setAuthenticated] = useState(false);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    setIsClient(true);

    const checkAuth = async () => {
      try {
        const response = await fetch('/api/auth/session');
        const data = await response.json();
        setAuthenticated(data.authenticated);
      } catch (error) {
        console.error('Error checking authentication:', error);
        setAuthenticated(false);
      } finally {
        setLoading(false);
      }
    };

    if (isClient) {
      checkAuth();
    } else {
      setLoading(false);
    }
  }, [isClient]);

  // Show fallback during SSR and initial client load
  if (!isClient || loading) {
    return fallback;
  }

  if (!authenticated) {
    // Redirect to sign-in page
    return (
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h2>Access Denied</h2>
              </div>
              <div className="card__body">
                <p>Please sign in to access this content.</p>
                <a href="/auth/signin" className="button button--primary">
                  Sign In
                </a>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  return <>{children}</>;
};
