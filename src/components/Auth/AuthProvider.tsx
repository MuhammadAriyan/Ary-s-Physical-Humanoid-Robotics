import React, { createContext, useContext, useEffect, useState, useCallback, ReactNode } from 'react';
import { useSession, AuthUser, AuthSession } from '../../lib/auth-client';

const AUTH_STORAGE_KEY = 'fubuni_auth_user';

interface AuthContextValue {
  user: AuthUser | null;
  session: AuthSession | null;
  isLoading: boolean;
  isAuthenticated: boolean;
  error: string | null;
  sessionExpired: boolean;
  refetch: () => void;
  clearSessionExpired: () => void;
  clearPersistedAuth: () => void;
}

const AuthContext = createContext<AuthContextValue | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const { data: sessionData, isPending, error, refetch } = useSession();
  const [authError, setAuthError] = useState<string | null>(null);
  const [sessionExpired, setSessionExpired] = useState(false);
  const [persistedUser, setPersistedUser] = useState<AuthUser | null>(null);

  // Load persisted user from localStorage on mount
  useEffect(() => {
    if (typeof window !== 'undefined') {
      try {
        const stored = localStorage.getItem(AUTH_STORAGE_KEY);
        if (stored) {
          const user = JSON.parse(stored);
          setPersistedUser(user);
        }
      } catch (e) {
        console.error('Failed to load persisted auth user:', e);
      }
    }
  }, []);

  // Save user to localStorage when session is valid
  useEffect(() => {
    if (sessionData?.user && typeof window !== 'undefined') {
      try {
        localStorage.setItem(AUTH_STORAGE_KEY, JSON.stringify(sessionData.user));
        setPersistedUser(sessionData.user);
      } catch (e) {
        console.error('Failed to persist auth user:', e);
      }
    }
  }, [sessionData]);

  // Clear persisted user on sign out
  const handleSignOut = useCallback(() => {
    if (typeof window !== 'undefined') {
      localStorage.removeItem(AUTH_STORAGE_KEY);
    }
    setPersistedUser(null);
    setSessionExpired(false);
    setAuthError(null);
  }, []);

  // Handle session expiry detection
  useEffect(() => {
    if (error) {
      const errorMessage = error.message || 'Authentication error';
      setAuthError(errorMessage);

      // Check if the error indicates session expiry
      if (
        errorMessage.toLowerCase().includes('expired') ||
        errorMessage.toLowerCase().includes('invalid session') ||
        errorMessage.toLowerCase().includes('unauthorized') ||
        errorMessage.toLowerCase().includes('jwt')
      ) {
        setSessionExpired(true);
      }
    } else {
      setAuthError(null);
      // If we have a valid session, clear the expired flag
      if (sessionData?.user) {
        setSessionExpired(false);
      }
    }
  }, [error, sessionData]);

  // Clear session expired flag (called after re-authentication)
  const clearSessionExpired = useCallback(() => {
    setSessionExpired(false);
    setAuthError(null);
  }, []);

  const value: AuthContextValue = {
    user: sessionData?.user || persistedUser || null,
    session: sessionData || null,
    isLoading: isPending,
    isAuthenticated: !!sessionData?.user || !!persistedUser,
    error: authError,
    sessionExpired,
    refetch,
    clearSessionExpired,
    clearPersistedAuth: handleSignOut,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = (): AuthContextValue => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export default AuthProvider;
