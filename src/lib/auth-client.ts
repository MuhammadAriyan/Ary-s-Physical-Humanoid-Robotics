import { createAuthClient } from "better-auth/react";

// Environment-aware auth service URL
const getAuthUrl = () => {
  if (typeof window !== "undefined") {
    // Client-side: check for production or use localhost
    if (window.location.hostname !== "localhost") {
      // Production: use Koyeb-deployed auth service
      return "https://gorgeous-deeanne-ary-s-88e09c71.koyeb.app";
    }
  }
  return "http://localhost:4000";
};

export const authClient = createAuthClient({
  baseURL: getAuthUrl(),
  fetchOptions: {
    credentials: "include", // Send cookies cross-origin
  },
});

// Export typed hooks and functions
export const {
  signIn,
  signUp,
  signOut,
  useSession,
  getSession,
} = authClient;

// New function: Fetch JWT token and store it in localStorage
export async function fetchAndStoreJWT(): Promise<string | null> {
  try {
    const response = await fetch(`${getAuthUrl()}/api/auth/token`, {
      credentials: 'include',
    });

    if (!response.ok) {
      console.warn('Failed to fetch JWT token:', response.status);
      return null;
    }

    const data = await response.json();
    if (data.token) {
      // Store JWT in localStorage
      localStorage.setItem('fubuni_jwt_token', data.token);
      localStorage.setItem('fubuni_jwt_timestamp', Date.now().toString());
      return data.token;
    }
    return null;
  } catch (error) {
    console.error('Error fetching JWT:', error);
    return null;
  }
}

// Helper function to get the current session token for API calls
// Uses stored JWT from localStorage (set during authentication)
export async function getSessionToken(): Promise<string | null> {
  try {
    // Check if we have a stored JWT token
    if (typeof window !== 'undefined') {
      const storedToken = localStorage.getItem('fubuni_jwt_token');
      const timestamp = localStorage.getItem('fubuni_jwt_timestamp');

      // Check if token exists and is less than 6 days old (7 day expiry with 1 day buffer)
      if (storedToken && timestamp) {
        const age = Date.now() - parseInt(timestamp);
        const sixDays = 6 * 24 * 60 * 60 * 1000;

        if (age < sixDays) {
          return storedToken;
        }
      }
    }

    // Try to fetch new token (will likely fail cross-domain, but worth trying)
    const hasLocalStorageAuth = typeof window !== 'undefined' &&
      localStorage.getItem('fubuni_auth_user') !== null;

    if (!hasLocalStorageAuth) {
      const session = await getSession();
      if (!session?.data?.user) {
        return null;
      }
    }

    const response = await fetch(`${getAuthUrl()}/api/auth/token`, {
      credentials: 'include',
    });

    if (!response.ok) {
      console.warn('Failed to fetch JWT token from auth service');
      return null;
    }

    const data = await response.json();
    if (data.token) {
      // Store for future use
      localStorage.setItem('fubuni_jwt_token', data.token);
      localStorage.setItem('fubuni_jwt_timestamp', Date.now().toString());
      return data.token;
    }

    return null;
  } catch (error) {
    console.error('Error getting session token:', error);
    return null;
  }
}

// Google sign-in helper function
export const signInWithGoogle = () => {
  // Don't await - this triggers a browser redirect to Google OAuth
  // The loading state will persist during redirect (expected behavior)
  signIn.social({
    provider: "google",
    // Use current page URL to redirect back to where user was
    callbackURL: window.location.href,
  });

  // Note: Code after this may or may not execute before redirect
  // Don't rely on any code here running to completion
};

// Types for auth state
export interface AuthUser {
  id: string;
  email?: string;
  name: string;
  image?: string;
  emailVerified: boolean;
  createdAt: Date;
  updatedAt: Date;
}

export interface AuthSession {
  user: AuthUser;
  session: {
    id: string;
    userId: string;
    expiresAt: Date;
    token: string;
  };
}
