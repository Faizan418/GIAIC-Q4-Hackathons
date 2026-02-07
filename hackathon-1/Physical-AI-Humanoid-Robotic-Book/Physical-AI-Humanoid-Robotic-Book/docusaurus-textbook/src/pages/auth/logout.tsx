import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';

function LogoutPage() {
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    // Mark as client-side to prevent SSR issues
    setIsClient(true);

    const performLogout = async () => {
      // Dynamically import the session context to avoid SSR issues
      const { useSession } = await import('../../services/auth/session-context');
      const { SessionProvider } = await import('../../services/auth/session-context');

      // Since we're in a page component, we need to get the context differently
      // We'll make the API call directly instead
      try {
        const response = await fetch('/api/auth/logout', {
          method: 'POST',
        });

        if (response.ok) {
          // Redirect to sign-in page after a short delay
          setTimeout(() => {
            window.location.href = '/auth/signin';
          }, 1500);
        } else {
          // If logout fails, still redirect to sign-in
          setTimeout(() => {
            window.location.href = '/auth/signin';
          }, 1500);
        }
      } catch (error) {
        console.error('Logout error:', error);
        // Redirect to sign-in even if there's an error
        setTimeout(() => {
          window.location.href = '/auth/signin';
        }, 1500);
      }
    };

    // Only run logout logic on client side
    if (isClient) {
      performLogout();
    }
  }, [isClient]);

  // Show a basic message during SSR, then handle logout on client
  if (!isClient) {
    return (
      <Layout title="Signing Out" description="Signing out from your account">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <div className="card">
                <div className="card__header">
                  <h2>Signing Out</h2>
                </div>
                <div className="card__body">
                  <p>Preparing to sign out...</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Signing Out" description="Signing out from your account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h2>Signing Out</h2>
              </div>
              <div className="card__body">
                <p>You are being signed out...</p>
                <p>Redirecting to sign in page...</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default LogoutPage;
