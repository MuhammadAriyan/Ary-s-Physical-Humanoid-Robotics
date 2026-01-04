// Vercel API route for auth service
import { toNodeHandler } from "better-auth/node";
import { auth } from "../src/auth.js";
import jwt from "jsonwebtoken";

// Parse CORS origins from environment
const corsOrigins = process.env.CORS_ORIGINS?.split(",").map((o) => o.trim()) || [
  "http://localhost:5000",
  "http://localhost:3000",
  "http://localhost:3001",
];

// Add GitHub Pages origin for production (if not already in env)
if (!corsOrigins.includes("https://muhammadariyan.github.io")) {
  corsOrigins.push("https://muhammadariyan.github.io");
}
if (!corsOrigins.includes("https://muhammadariyan.github.io/Ary-s-Physical-Humanoid-Robotics")) {
  corsOrigins.push("https://muhammadariyan.github.io/Ary-s-Physical-Humanoid-Robotics");
}

// Add Replit URL patterns for Replit deployment
if (process.env.REPL_SLUG && process.env.REPL_OWNER) {
  const replitUrl = `https://${process.env.REPL_SLUG}.${process.env.REPL_OWNER}.repl.co`;
  if (!corsOrigins.includes(replitUrl)) {
    corsOrigins.push(replitUrl);
  }
}

// Simple CORS check
function checkCors(origin) {
  if (!origin) return true;
  return corsOrigins.includes(origin);
}

// Create the Vercel API handler
export default async function handler(req, res) {
  const { method, url } = req;

  // Handle CORS preflight
  if (method === 'OPTIONS') {
    const origin = req.headers.origin;
    const isAllowed = checkCors(origin);

    res.setHeader('Access-Control-Allow-Origin', isAllowed ? origin : 'false');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization, X-Requested-With');
    res.setHeader('Access-Control-Allow-Credentials', 'true');

    return res.status(200).end();
  }

  // Add CORS headers for all responses
  const origin = req.headers.origin;
  const isAllowed = checkCors(origin);
  if (isAllowed) {
    res.setHeader('Access-Control-Allow-Origin', origin);
    res.setHeader('Access-Control-Allow-Credentials', 'true');
  }

  try {
    // Handle different routes
    if (url === '/health' && method === 'GET') {
      return res.json({
        status: "healthy",
        service: "fubuni-auth-service",
        timestamp: new Date().toISOString(),
        replit: {
          slug: process.env.REPL_SLUG || null,
          owner: process.env.REPL_OWNER || null,
          isReplit: !!(process.env.REPL_SLUG && process.env.REPL_OWNER)
        }
      });
    }

    if (url === '/wake-up' && method === 'GET') {
      return res.json({
        status: "awake",
        message: "Service is awake and responsive",
        timestamp: new Date().toISOString()
      });
    }

    if (url === '/test/auth-flow' && method === 'GET') {
      return res.json({
        timestamp: new Date().toISOString(),
        replit: {
          slug: process.env.REPL_SLUG || null,
          owner: process.env.REPL_OWNER || null,
          isReplit: !!(process.env.REPL_SLUG && process.env.REPL_OWNER)
        },
        tests: {
          serverConfig: {
            baseUrl: process.env.BETTER_AUTH_URL || "http://localhost:5000",
            isReplitEnv: !!(process.env.REPL_SLUG && process.env.REPL_OWNER)
          },
          corsConfig: {
            allowedOrigins: corsOrigins,
            currentOrigin: req.headers.origin || req.headers.referer || 'none'
          },
          endpoints: {
            healthCheck: true,
            authEndpointsMounted: true,
            wakeUpEndpoint: true
          },
          environment: {
            nodeEnv: process.env.NODE_ENV,
            hasDatabase: !!process.env.NEON_DATABASE_URL,
            hasAuthSecret: !!process.env.BETTER_AUTH_SECRET
          }
        }
      });
    }

    if (url === '/debug/cors' && method === 'GET') {
      return res.json({
        corsOrigins,
        envCorsOrigins: process.env.CORS_ORIGINS,
        envCorsOrigins2: process.env.CORS_ORIGINS2,
        nodeEnv: process.env.NODE_ENV,
        betterAuthUrl: process.env.BETTER_AUTH_URL,
      });
    }

    if (url === '/api/auth/token' && method === 'GET') {
      try {
        // Convert headers to plain object for Better Auth
        const headers = {};
        for (const [key, value] of Object.entries(req.headers)) {
          if (typeof value === 'string') {
            headers[key] = value;
          }
        }

        // Get the session using Better Auth
        const session = await auth.api.getSession({
          headers: headers,
        });

        if (!session?.user) {
          return res.status(401).json({ error: "Not authenticated" });
        }

        const JWT_SECRET = process.env.JWT_SECRET || "default-secret-change-in-production";

        // Create JWT token with user info
        const token = jwt.sign(
          {
            userId: session.user.id,
            email: session.user.email,
            name: session.user.name,
            emailVerified: session.user.emailVerified,
          },
          JWT_SECRET,
          { expiresIn: "7d" }
        );

        return res.json({ token });
      } catch (error) {
        console.error("Token generation error:", error);
        return res.status(500).json({ error: "Failed to generate token" });
      }
    }

    // Handle Better Auth routes
    if (url.startsWith('/api/auth/')) {
      return toNodeHandler(auth)(req, res);
    }

    // 404 for unknown routes
    res.status(404).json({ error: "Not Found" });
  } catch (error) {
    console.error("API route error:", error);
    res.status(500).json({
      error: "Internal server error",
      message: process.env.NODE_ENV === "development" ? error.message : undefined
    });
  }
}

export const config = {
  api: {
    externalResolver: true,
  },
};