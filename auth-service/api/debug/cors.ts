// CORS debug endpoint for Vercel
export default function handler(req, res) {
  if (req.method === 'OPTIONS') {
    // Handle CORS preflight
    const origin = req.headers.origin;
    res.setHeader('Access-Control-Allow-Origin', origin || '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization, X-Requested-With');
    res.setHeader('Access-Control-Allow-Credentials', 'true');
    return res.status(200).end();
  }

  // Add CORS headers
  const origin = req.headers.origin;
  res.setHeader('Access-Control-Allow-Origin', origin || '*');
  res.setHeader('Access-Control-Allow-Credentials', 'true');

  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  // Parse CORS origins from environment
  const corsOrigins = process.env.CORS_ORIGINS?.split(",").map((o) => o.trim()) || [
    "http://localhost:5000",
    "http://localhost:3000",
    "http://localhost:3001",
  ];

  res.json({
    corsOrigins,
    envCorsOrigins: process.env.CORS_ORIGINS,
    envCorsOrigins2: process.env.CORS_ORIGINS2,
    nodeEnv: process.env.NODE_ENV,
    betterAuthUrl: process.env.BETTER_AUTH_URL,
  });
}

export const config = {
  api: {
    bodyParser: false,
  },
};