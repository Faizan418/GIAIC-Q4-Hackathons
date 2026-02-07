import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { ProtectedRoute } from '../services/auth/protected-route';

function DashboardPage() {
  const [isClient, setIsClient] = useState(false);
  const [userData, setUserData] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    setIsClient(true);

    // Fetch user data directly instead of using the hook to avoid SSR issues
    const fetchUserData = async () => {
      try {
        const response = await fetch('/api/auth/session');
        const data = await response.json();

        if (data.authenticated && data.user) {
          setUserData(data.user);
        }
      } catch (error) {
        console.error('Error fetching user data:', error);
      } finally {
        setLoading(false);
      }
    };

    if (isClient) {
      fetchUserData();
    } else {
      setLoading(false);
    }
  }, [isClient]);

  // Check if profile is incomplete
  const isProfileIncomplete = !userData?.softwareBackground && !userData?.hardwareExperience;

  // Show loading state during SSR and initial client load
  if (!isClient || loading) {
    return (
      <Layout title="Dashboard" description="Your personalized dashboard">
        <ProtectedRoute>
          <div className="container margin-vert--lg">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <div className="card">
                  <div className="card__header">
                    <h2>Loading Dashboard...</h2>
                  </div>
                  <div className="card__body">
                    <p>Loading your personalized dashboard...</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </ProtectedRoute>
      </Layout>
    );
  }

  return (
    <Layout title="Dashboard" description="Your personalized dashboard">
      <ProtectedRoute>
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className="card">
                <div className="card__header">
                  <h2>Welcome{userData?.name ? ` ${userData.name.split(' ')[0]}` : ''}!</h2>
                </div>
                <div className="card__body">
                  {isProfileIncomplete && (
                    <div className="alert alert--warning margin-bottom--lg" role="alert">
                      <h4>Complete Your Profile</h4>
                      <p>Your profile is not yet complete. Consider updating your background information to get the most personalized experience.</p>
                      <a href="/auth/signup" className="button button--primary button--sm">
                        Update Profile
                      </a>
                    </div>
                  )}

                  <div className="margin-bottom--lg">
                    <h3>Profile Information</h3>
                    <div className="row">
                      <div className="col col--6">
                        <p><strong>Email:</strong> {userData?.email}</p>
                        <p><strong>Name:</strong> {userData?.name || 'Not provided'}</p>
                      </div>
                      <div className="col col--6">
                        <p><strong>Member since:</strong> {userData ? new Date(userData.createdAt).toLocaleDateString() : ''}</p>
                      </div>
                    </div>
                  </div>

                  <div className="margin-bottom--lg">
                    <h3>Background Information</h3>
                    <div className="row">
                      <div className="col col--6">
                        <p><strong>Software Background:</strong></p>
                        <p className="text--gray">{userData?.softwareBackground || 'Not provided'}</p>
                      </div>
                      <div className="col col--6">
                        <p><strong>Hardware Experience:</strong></p>
                        <p className="text--gray">{userData?.hardwareExperience || 'Not provided'}</p>
                      </div>
                    </div>

                    <div className="row margin-top--md">
                      <div className="col col--6">
                        <p><strong>Learning Track:</strong> {userData?.learningTrack || 'Not provided'}</p>
                      </div>
                      <div className="col col--6">
                        <p><strong>Skill Level:</strong> {userData?.skillLevel || 'Not provided'}</p>
                      </div>
                    </div>
                  </div>

                  <div className="margin-top--lg">
                    <a href="/auth/logout" className="button button--outline button--secondary">
                      Sign Out
                    </a>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </ProtectedRoute>
    </Layout>
  );
}

export default DashboardPage;
