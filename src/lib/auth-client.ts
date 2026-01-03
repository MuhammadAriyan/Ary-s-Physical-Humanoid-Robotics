import { createAuthClient } from "better-auth/react";

// Environment-aware auth service URL
const getAuthUrl = () => {
  if (typeof window !== "undefined") {
    // Client-side: check for production or use localhost
    if (window.location.hostname !== "localhost") {
      // Production: use Koyeb-deployed auth service
      return "https://ary-s-physical-humanoid-robotics--maryanrar.replit.app";
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
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      // Only warn for errors other than 401, since 401 is expected when not authenticated
      if (response.status !== 401) {
        console.warn('Failed to fetch JWT token:', response.status, response.statusText);
      }
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
    console.warn('Network error fetching JWT:', error);
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

    // Check if user is authenticated by trying to get the session from auth service
    try {
      const sessionResponse = await fetch(`${getAuthUrl()}/api/auth/session`, {
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!sessionResponse.ok) {
        // If session endpoint returns 401, user is not authenticated
        if (sessionResponse.status === 401) {
          if (typeof window !== 'undefined') {
            localStorage.removeItem('fubuni_jwt_token');
            localStorage.removeItem('fubuni_jwt_timestamp');
          }
          return null;
        }
        // If other error, return null to avoid using invalid tokens
        console.warn('Session check failed:', sessionResponse.status);
        return null;
      } else {
        // If session exists, try to get JWT token
        const jwtResponse = await fetch(`${getAuthUrl()}/api/auth/token`, {
          credentials: 'include',
          headers: {
            'Content-Type': 'application/json',
          },
        });

        if (jwtResponse.ok) {
          const data = await jwtResponse.json();
          if (data.token) {
            // Store for future use
            localStorage.setItem('fubuni_jwt_token', data.token);
            localStorage.setItem('fubuni_jwt_timestamp', Date.now().toString());
            return data.token;
          }
        } else {
          // Don't warn about 401 here since it's expected when session exists but JWT generation fails
          if (jwtResponse.status !== 401) {
            console.warn('Failed to get JWT token:', jwtResponse.status, jwtResponse.statusText);
          }
          return null;
        }
      }
    } catch (sessionError) {
      console.warn('Network error checking session:', sessionError);
      return null;
    }

    return null;
  } catch (error) {
    console.warn('Error getting session token:', error);
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

// Debug function to check session status
export async function checkSessionStatus(): Promise<{ authenticated: boolean; hasJwt: boolean; jwtAge?: number }> {
  try {
    // Check if we have a stored JWT token
    const storedToken = localStorage.getItem('fubuni_jwt_token');
    const timestamp = localStorage.getItem('fubuni_jwt_timestamp');

    let hasJwt = false;
    let jwtAge = 0;
    if (storedToken && timestamp) {
      hasJwt = true;
      jwtAge = Date.now() - parseInt(timestamp);
    }

    // Try to check session with auth service
    const sessionResponse = await fetch(`${getAuthUrl()}/api/auth/session`, {
      credentials: 'include',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    const isAuthenticated = sessionResponse.ok && sessionResponse.status !== 401;

    return {
      authenticated: isAuthenticated,
      hasJwt,
      jwtAge
    };
  } catch (error) {
    console.error('Error checking session status:', error);
    return {
      authenticated: false,
      hasJwt: false
    };
  }
}

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
