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

// Helper function to get the current session token for API calls
// We call the auth service to generate a JWT token from the Better Auth session
export async function getSessionToken(): Promise<string | null> {
  try {
    // Check if user is authenticated first
    const session = await getSession();
    if (!session?.data?.user) {
      return null;
    }

    // Get JWT token from auth service
    const response = await fetch(`${getAuthUrl()}/api/auth/token`, {
      credentials: 'include', // Include cookies for Better Auth session
    });

    if (!response.ok) {
      return null;
    }

    const data = await response.json();
    return data.token || null;
  } catch {
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
