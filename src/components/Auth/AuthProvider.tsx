import React, { createContext, useContext, useEffect, useState, useCallback, ReactNode } from 'react';
import { useSession, AuthUser, AuthSession } from '../../lib/auth-client';

interface AuthContextValue {
  user: AuthUser | null;
  session: AuthSession | null;
  isLoading: boolean;
  isAuthenticated: boolean;
  error: string | null;
  sessionExpired: boolean;
  refetch: () => void;
  clearSessionExpired: () => void;
}

const AuthContext = createContext<AuthContextValue | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const { data: sessionData, isPending, error, refetch } = useSession();
  const [authError, setAuthError] = useState<string | null>(null);
  const [sessionExpired, setSessionExpired] = useState(false);

  // Handle session expiry detection
  useEffect(() => {
    if (error) {
      const errorMessage = error.message || 'Authentication error';
      setAuthError(errorMessage);

      // Check if the error indicates session expiry
      if (
        errorMessage.toLowerCase().includes('expired') ||
        errorMessage.toLowerCase().includes('invalid session') ||
        errorMessage.toLowerCase().includes('unauthorized')
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
    user: sessionData?.user || null,
    session: sessionData || null,
    isLoading: isPending,
    isAuthenticated: !!sessionData?.user,
    error: authError,
    sessionExpired,
    refetch,
    clearSessionExpired,
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
