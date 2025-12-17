import { createAuthClient } from "better-auth/react";

// Environment-aware auth service URL
const getAuthUrl = () => {
  // Check for build-time environment variable first (set during GitHub Actions build)
  if (process.env.REACT_APP_AUTH_URL) {
    return process.env.REACT_APP_AUTH_URL;
  }

  if (typeof window !== "undefined") {
    // Client-side: check for production or use localhost
    if (window.location.hostname !== "localhost") {
      // Production: fallback to deployed auth service on Vercel
      return "https://ary-s-physical-humanoid-robotics-lrmuq08ij.vercel.app";
    }
  }
  return "http://localhost:4000";
};

export const authClient = createAuthClient({
  baseURL: getAuthUrl(),
  fetchOptions: {
    credentials: "include", // Required for cross-origin cookies
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

// Google sign-in helper function
export const signInWithGoogle = async () => {
  try {
    const result = await signIn.social({
      provider: "google",
      callbackURL: window.location.origin,
    });
    return result;
  } catch (error) {
    console.error("Google sign-in error:", error);
    throw error;
  }
};

// Types for auth state
export interface AuthUser {
  id: string;
  email: string;
  name: string | null;
  image: string | null;
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
