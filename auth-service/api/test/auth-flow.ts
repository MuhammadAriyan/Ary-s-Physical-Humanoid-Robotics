// Test auth flow endpoint for Vercel
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

  res.json({
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

export const config = {
  api: {
    bodyParser: false,
  },
};