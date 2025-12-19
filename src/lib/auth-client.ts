import { createAuthClient } from "better-auth/react";

// Environment-aware auth service URL
const getAuthUrl = () => {
  if (typeof window !== "undefined") {
    // Client-side: check for production or use localhost
    if (window.location.hostname !== "localhost") {
      // Production: use relative path or configured URL
      return process.env.REACT_APP_AUTH_URL || "https://your-auth-service.hf.space";
    }
  }
  return "http://localhost:4000";
};

export const authClient = createAuthClient({
  baseURL: getAuthUrl(),
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
