import React, { createContext, useContext, useEffect, useState, useCallback, ReactNode, Component, ErrorInfo } from 'react';
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

// Default context value when auth is unavailable
const defaultAuthValue: AuthContextValue = {
  user: null,
  session: null,
  isLoading: false,
  isAuthenticated: false,
  error: null,
  sessionExpired: false,
  refetch: () => {},
  clearSessionExpired: () => {},
};

interface AuthProviderProps {
  children: ReactNode;
}

// Error boundary to catch auth initialization errors
interface ErrorBoundaryState {
  hasError: boolean;
}

class AuthErrorBoundary extends Component<{ children: ReactNode; fallback: ReactNode }, ErrorBoundaryState> {
  constructor(props: { children: ReactNode; fallback: ReactNode }) {
    super(props);
    this.state = { hasError: false };
  }

  static getDerivedStateFromError(): ErrorBoundaryState {
    return { hasError: true };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    console.warn('Auth initialization error (non-critical):', error.message);
  }

  render() {
    if (this.state.hasError) {
      return this.props.fallback;
    }
    return this.props.children;
  }
}

// Inner provider that uses useSession
const AuthProviderInner: React.FC<AuthProviderProps> = ({ children }) => {
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

// Fallback provider when auth service is unavailable
const AuthProviderFallback: React.FC<AuthProviderProps> = ({ children }) => {
  return (
    <AuthContext.Provider value={defaultAuthValue}>
      {children}
    </AuthContext.Provider>
  );
};

// Main export with error boundary
export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  return (
    <AuthErrorBoundary fallback={<AuthProviderFallback>{children}</AuthProviderFallback>}>
      <AuthProviderInner>{children}</AuthProviderInner>
    </AuthErrorBoundary>
  );
};

export const useAuth = (): AuthContextValue => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    // Return default value instead of throwing - allows usage outside provider
    return defaultAuthValue;
  }
  return context;
};

export default AuthProvider;
