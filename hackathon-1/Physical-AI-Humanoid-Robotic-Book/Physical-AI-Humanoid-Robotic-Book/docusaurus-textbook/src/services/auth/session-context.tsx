import { createContext, useContext, useEffect, useState, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  name?: string;
  createdAt: string;
  updatedAt: string;
  // Custom metadata fields
  softwareBackground?: string;
  hardwareExperience?: string;
  learningTrack?: string;
  skillLevel?: string;
}

interface AuthSession {
  id: string;
  userId: string;
  expiresAt: string;
  createdAt: string;
}

interface SessionContextType {
  user: User | null;
  session: AuthSession | null;
  authenticated: boolean;
  loading: boolean;
  checkSession: () => Promise<void>;
  signOut: () => Promise<void>;
}

const SessionContext = createContext<SessionContextType | undefined>(undefined);

export const SessionProvider = ({ children }: { children: ReactNode }) => {
  const [user, setUser] = useState<User | null>(null);
  const [session, setSession] = useState<AuthSession | null>(null);
  const [authenticated, setAuthenticated] = useState(false);
  const [loading, setLoading] = useState(true);
  const [mounted, setMounted] = useState(false);

  const checkSession = async () => {
    try {
      const response = await fetch('/api/auth/session');
      const data = await response.json();

      if (data.authenticated) {
        setUser(data.user);
        setSession(data.session);
        setAuthenticated(true);
      } else {
        setUser(null);
        setSession(null);
        setAuthenticated(false);
      }
    } catch (error) {
      console.error('Error checking session:', error);
      setUser(null);
      setSession(null);
      setAuthenticated(false);
    } finally {
      setLoading(false);
    }
  };

  const signOut = async () => {
    try {
      const response = await fetch('/api/auth/logout', {
        method: 'POST',
      });

      if (response.ok) {
        setUser(null);
        setSession(null);
        setAuthenticated(false);
      }
    } catch (error) {
      console.error('Error signing out:', error);
    }
  };

  useEffect(() => {
    setMounted(true); // Mark as mounted on client side

    // Only run on client side
    if (typeof window !== 'undefined') {
      // Check session status on component mount
      checkSession();

      // Add event listener to check session on page visibility change
      const handleVisibilityChange = () => {
        if (!document.hidden) {
          checkSession();
        }
      };

      document.addEventListener('visibilitychange', handleVisibilityChange);

      return () => {
        document.removeEventListener('visibilitychange', handleVisibilityChange);
      };
    } else {
      // On server, set loading to false to avoid hydration issues
      setLoading(false);
    }
  }, []);

  // Don't render children until mounted to avoid build-time issues
  if (!mounted) {
    return <>{children}</>;
  }

  return (
    <SessionContext.Provider
      value={{
        user,
        session,
        authenticated,
        loading,
        checkSession,
        signOut,
      }}
    >
      {children}
    </SessionContext.Provider>
  );
};

export const useSession = () => {
  const context = useContext(SessionContext);
  if (context === undefined) {
    throw new Error('useSession must be used within a SessionProvider');
  }
  return context;
};
