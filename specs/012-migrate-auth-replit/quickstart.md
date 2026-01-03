# Quickstart: Auth Service on Replit

## Prerequisites
- Replit account
- Existing Better Auth + Express service code
- Environment variables/secrets ready

## Setup Steps

### 1. Create Replit Project
1. Go to [replit.com](https://replit.com)
2. Click "Create" to make a new repl
3. Choose "Node.js" environment
4. Name your repl appropriately (e.g., "auth-service")

### 2. Upload Your Code
1. Upload your existing auth service files to Replit
2. Ensure your main file is properly configured to run on Replit
3. Verify package.json has correct dependencies and start script

### 3. Update Server Configuration
Update your main server file (typically `index.js`) to work with Replit:

```javascript
import express from 'express';
import { betterAuth } from '@better-auth/node';

const app = express();

// Your Better Auth configuration
app.use(betterAuth({
  // your existing config
}));

// Listen on Replit's PORT environment variable
const port = process.env.PORT || 3000;
app.listen(port, '0.0.0.0', () => {
  console.log(`Server running on port ${port}`);
});
```

### 4. Configure Environment Variables
1. In Replit, go to "Secrets" or "Environment Variables"
2. Add all necessary environment variables:
   - BETTER_AUTH_SECRET
   - BETTER_AUTH_URL (set to your Replit URL)
   - DATABASE_URL
   - OAuth provider credentials (GITHUB_ID, GITHUB_SECRET, etc.)

### 5. Update CORS Configuration
In your Better Auth configuration, ensure the baseURL matches your Replit URL:

```javascript
betterAuth({
  baseURL: 'https://your-repl-name.your-username.repl.co', // Replit URL
  // ... other config
  cors: {
    origins: [
      'https://your-frontend-domain.com', // Your frontend URL
      'http://localhost:3000', // For local development
    ],
  },
})
```

### 6. Run the Service
1. Click the "Run" button in Replit
2. Monitor the console for any errors
3. Test the service endpoints

## Testing
- Visit your Replit URL to verify the service is running
- Test authentication endpoints
- Verify CORS allows requests from your frontend applications

## Troubleshooting
- If the service doesn't start, check the console for errors
- If CORS is blocking requests, verify your origins configuration
- If authentication fails, check that all environment variables are properly set