import express from "express";
import cors from "cors";
import dotenv from "dotenv";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth.js";
import { testConnection, closePool } from "./db.js";

dotenv.config();

const app = express();
const PORT = process.env.AUTH_PORT || 4000;

// Parse CORS origins from environment
const corsOrigins = process.env.CORS_ORIGINS?.split(",").map((o) => o.trim()) || [
  "http://localhost:3000",
  "http://localhost:3001",
];

// CORS configuration
app.use(
  cors({
    origin: corsOrigins,
    credentials: true,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);

// Parse JSON bodies
app.use(express.json());

// Health check endpoint
app.get("/health", (_req, res) => {
  res.json({
    status: "healthy",
    service: "fubuni-auth-service",
    timestamp: new Date().toISOString(),
  });
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
    console.error("Server error:", err);
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

  app.listen(PORT, () => {
    console.log(`Auth service running on http://localhost:${PORT}`);
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
