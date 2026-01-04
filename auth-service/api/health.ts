// Health check endpoint for Vercel
export default function handler(req, res) {
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
  // Add Vercel deployment origin for production
  if (!corsOrigins.includes("https://ary-s-physical-humanoid-robotics-au.vercel.app")) {
    corsOrigins.push("https://ary-s-physical-humanoid-robotics-au.vercel.app");
  }

  // Add Replit URL patterns for Replit deployment
  if (process.env.REPL_SLUG && process.env.REPL_OWNER) {
    const replitUrl = `https://${process.env.REPL_SLUG}.${process.env.REPL_OWNER}.repl.co`;
    if (!corsOrigins.includes(replitUrl)) {
      corsOrigins.push(replitUrl);
    }
  }

  if (req.method === 'OPTIONS') {
    // Handle CORS preflight
    const origin = req.headers.origin;
    const isAllowed = corsOrigins.includes(origin);
    res.setHeader('Access-Control-Allow-Origin', isAllowed ? origin : 'false');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization, X-Requested-With');
    res.setHeader('Access-Control-Allow-Credentials', 'true');
    return res.status(200).end();
  }

  // Add CORS headers
  const origin = req.headers.origin;
  const isAllowed = corsOrigins.includes(origin);
  if (isAllowed) {
    res.setHeader('Access-Control-Allow-Origin', origin);
    res.setHeader('Access-Control-Allow-Credentials', 'true');
  }

  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  res.json({
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

export const config = {
  api: {
    bodyParser: false,
  },
};