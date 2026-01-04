// JWT token generation endpoint for Vercel
import { auth } from "../../../src/auth.js";
import jwt from "jsonwebtoken";

export default async function handler(req, res) {
  if (req.method === 'OPTIONS') {
    // Handle CORS preflight
    const origin = req.headers.origin;
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

    const isAllowed = corsOrigins.includes(origin);
    res.setHeader('Access-Control-Allow-Origin', isAllowed ? origin : 'false');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization, X-Requested-With');
    res.setHeader('Access-Control-Allow-Credentials', 'true');
    return res.status(200).end();
  }

  // Add CORS headers
  const origin = req.headers.origin;
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

  const isAllowed = corsOrigins.includes(origin);
  if (isAllowed) {
    res.setHeader('Access-Control-Allow-Origin', origin);
    res.setHeader('Access-Control-Allow-Credentials', 'true');
  }

  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

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

    res.json({ token });
  } catch (error) {
    console.error("Token generation error:", error);
    res.status(500).json({ error: "Failed to generate token" });
  }
}

export const config = {
  api: {
    bodyParser: false,
  },
};