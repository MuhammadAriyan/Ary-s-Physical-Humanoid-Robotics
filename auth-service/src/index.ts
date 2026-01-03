import express from "express";
import cors from "cors";
import dotenv from "dotenv";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth.js";
import { testConnection, closePool } from "./db.js";
import jwt from "jsonwebtoken";

dotenv.config();

const app = express();
const PORT = process.env.PORT || process.env.AUTH_PORT || 5000;
const JWT_SECRET = process.env.JWT_SECRET || "default-secret-change-in-production";

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

// Parse JSON bodies first
app.use(express.json());

// CORS configuration - use function to allow both base and full path
app.use(
  cors({
    origin: (origin, callback) => {
      if (!origin) return callback(null, true);

      // Enhanced CORS logging for debugging
      const isAllowed = corsOrigins.includes(origin);
      const corsLog = {
        timestamp: new Date().toISOString(),
        type: "CORS_CHECK",
        origin: origin,
        allowedOrigins: corsOrigins,
        isAllowed: isAllowed,
        replit: {
          slug: process.env.REPL_SLUG,
          owner: process.env.REPL_OWNER,
          isReplit: !!(process.env.REPL_SLUG && process.env.REPL_OWNER)
        }
      };
      console.log(corsLog);

      if (isAllowed) {
        return callback(null, true);
      } else {
        callback(null, false);
      }
    },
    credentials: true,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization", "X-Requested-With"],
    optionsSuccessStatus: 200,
  })
);

// Health check endpoint
app.get("/health", (_req, res) => {
  res.json({
    status: "healthy",
    service: "fubuni-auth-service",
    timestamp: new Date().toISOString(),
    uptime: process.uptime(),
    replit: {
      slug: process.env.REPL_SLUG || null,
      owner: process.env.REPL_OWNER || null,
      isReplit: !!(process.env.REPL_SLUG && process.env.REPL_OWNER)
    }
  });
});

// Replit-specific wake-up endpoint to prevent sleep
app.get("/wake-up", (_req, res) => {
  res.json({
    status: "awake",
    message: "Service is awake and responsive",
    timestamp: new Date().toISOString()
  });
});

// Comprehensive test endpoint for authentication flow verification
app.get("/test/auth-flow", async (req, res) => {
  try {
    const testResults = {
      timestamp: new Date().toISOString(),
      replit: {
        slug: process.env.REPL_SLUG || null,
        owner: process.env.REPL_OWNER || null,
        isReplit: !!(process.env.REPL_SLUG && process.env.REPL_OWNER)
      },
      tests: {
        serverConfig: {
          port: process.env.PORT || process.env.AUTH_PORT || 5000,
          baseUrl: process.env.BETTER_AUTH_URL || "http://localhost:5000",
          isReplitEnv: !!(process.env.REPL_SLUG && process.env.REPL_OWNER)
        },
        corsConfig: {
          allowedOrigins: corsOrigins,
          currentOrigin: req.get('Origin') || req.get('Referer') || 'none'
        },
        endpoints: {
          healthCheck: true, // This endpoint is accessible
          authEndpointsMounted: true, // Better Auth routes are mounted
          wakeUpEndpoint: true // Wake-up endpoint is accessible
        },
        environment: {
          nodeEnv: process.env.NODE_ENV,
          hasDatabase: !!process.env.NEON_DATABASE_URL,
          hasAuthSecret: !!process.env.BETTER_AUTH_SECRET
        }
      }
    };

    res.json(testResults);
  } catch (error) {
    console.error("Test endpoint error:", error);
    res.status(500).json({
      error: "Test endpoint failed",
      message: error instanceof Error ? error.message : "Unknown error"
    });
  }
});

// Debug endpoint to check CORS configuration
app.get("/debug/cors", (_req, res) => {
  res.json({
    corsOrigins,
    envCorsOrigins: process.env.CORS_ORIGINS,
    envCorsOrigins2: process.env.CORS_ORIGINS2,
    nodeEnv: process.env.NODE_ENV,
    betterAuthUrl: process.env.BETTER_AUTH_URL,
  });
});

// Custom endpoint to generate JWT token for authenticated sessions
// Frontend calls this to get a JWT that can be sent to FastAPI backend
app.get("/api/auth/token", async (req, res) => {
  try {
    // Convert Express headers to plain object for Better Auth
    const headers: Record<string, string> = {};
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

    res.json({ token });
  } catch (error) {
    console.error("Token generation error:", error);
    res.status(500).json({ error: "Failed to generate token" });
  }
});

// Mount Better Auth handler at /api/auth/*
app.all("/api/auth/*", toNodeHandler(auth));

// Error handling middleware
app.use(
  (
    err: Error,
    _req: express.Request,
    res: express.Response,
    _next: express.NextFunction
  ) => {
    // Log errors with Replit context
    console.error({
      timestamp: new Date().toISOString(),
      error: "Server error:",
      message: err.message,
      stack: process.env.NODE_ENV === "development" ? err.stack : undefined,
      replit: {
        slug: process.env.REPL_SLUG,
        owner: process.env.REPL_OWNER,
        isReplit: !!(process.env.REPL_SLUG && process.env.REPL_OWNER)
      }
    });

    res.status(500).json({
      error: "Internal server error",
      message: process.env.NODE_ENV === "development" ? err.message : undefined,
    });
  }
);

// Start server
async function start() {
  // Test database connection
  const dbConnected = await testConnection();
  if (!dbConnected) {
    console.error("Failed to connect to database. Exiting.");
    process.exit(1);
  }

  app.listen(PORT, '0.0.0.0', () => {
    console.log(`Auth service running on port ${PORT}`);
    console.log(`Health check: http://localhost:${PORT}/health`);
    console.log(`Auth endpoints: http://localhost:${PORT}/api/auth/*`);
  });
}

// Graceful shutdown
process.on("SIGTERM", async () => {
  console.log("SIGTERM received. Shutting down gracefully.");
  await closePool();
  process.exit(0);
});

process.on("SIGINT", async () => {
  console.log("SIGINT received. Shutting down gracefully.");
  await closePool();
  process.exit(0);
});

start().catch((error) => {
  console.error("Failed to start server:", error);
  process.exit(1);
});
