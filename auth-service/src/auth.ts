import { betterAuth } from "better-auth";
import { pool } from "./db.js";
import dotenv from "dotenv";

dotenv.config();

// Parse CORS origins from environment
const corsOrigins = process.env.CORS_ORIGINS?.split(",").map((o) => o.trim()) || [
  "http://localhost:3000",
  "http://localhost:3001",
];

// Check if Google OAuth is configured
const googleClientId = process.env.GOOGLE_CLIENT_ID;
const googleClientSecret = process.env.GOOGLE_CLIENT_SECRET;
const isGoogleConfigured = googleClientId && googleClientSecret;

export const auth = betterAuth({
  database: pool,

  // Base URL for auth service
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:4000",

  // Secret for signing tokens
  secret: process.env.BETTER_AUTH_SECRET,

  // Custom table names with ba_ prefix to avoid conflicts
  user: {
    modelName: "ba_user",
  },
  session: {
    modelName: "ba_session",
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Update session every 24 hours
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes
    },
  },
  account: {
    modelName: "ba_account",
    // Automatically link accounts with the same verified email
    accountLinking: {
      enabled: true,
      trustedProviders: ["google"],
    },
  },
  verification: {
    modelName: "ba_verification",
  },

  // Email/Password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Can enable later for production
  },

  // Social providers
  socialProviders: {
    // Google OAuth (only if credentials are configured)
    ...(isGoogleConfigured && {
      google: {
        clientId: googleClientId!,
        clientSecret: googleClientSecret!,
      },
    }),
  },

  // Trusted origins for CORS
  trustedOrigins: corsOrigins,

  // Advanced configuration
  advanced: {},
});

export type Auth = typeof auth;
