# Quickstart: Better Auth Integration

**Feature**: 008-better-auth
**Date**: 2025-12-17

## Prerequisites

Before starting implementation, ensure you have:

1. **Google Cloud Console Setup** (for Google OAuth)
   - Create project at https://console.cloud.google.com
   - Enable "Google+ API" or "Google Identity"
   - Create OAuth 2.0 credentials (Web application)
   - Add authorized redirect URI: `http://localhost:4000/api/auth/callback/google`
   - Note: Client ID and Client Secret

2. **Existing Services Running**
   - Neon PostgreSQL database accessible
   - FastAPI backend working (`http://localhost:8000`)
   - Frontend development server (`http://localhost:3000`)

3. **Tools Installed**
   - Node.js 20+ (for auth-service)
   - npm or pnpm
   - Python 3.11+ (existing)

## Step 1: Auth Service Setup

### 1.1 Create Directory Structure

```bash
mkdir -p auth-service/src
cd auth-service
```

### 1.2 Initialize Package

```bash
npm init -y
npm install better-auth express pg cors dotenv
npm install -D typescript @types/express @types/cors @types/pg tsx
```

### 1.3 Create Configuration Files

**tsconfig.json**:
```json
{
  "compilerOptions": {
    "target": "ES2022",
    "module": "ESNext",
    "moduleResolution": "bundler",
    "esModuleInterop": true,
    "strict": true,
    "outDir": "dist",
    "rootDir": "src"
  },
  "include": ["src/**/*"]
}
```

**.env.example**:
```env
AUTH_PORT=4000
NODE_ENV=development
NEON_DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
BETTER_AUTH_SECRET=your-32-character-secret-key-here
BETTER_AUTH_URL=http://localhost:4000
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret
```

### 1.4 Create Source Files

**src/auth.ts**: Better Auth configuration
**src/index.ts**: Express server

### 1.5 Run Database Migration

```bash
npx @better-auth/cli migrate
```

### 1.6 Start Auth Service

```bash
npm run dev
```

Verify at: `http://localhost:4000/health`

## Step 2: Frontend Integration

### 2.1 Install Dependencies

```bash
# From repository root
npm install better-auth
```

### 2.2 Create Auth Client

Create `src/lib/auth-client.ts` with Better Auth React client.

### 2.3 Create Components

- `src/components/Auth/AuthProvider.tsx` - Context provider
- `src/components/Auth/AuthModal.tsx` - Login/signup UI

### 2.4 Update Root.tsx

Wrap app with AuthProvider.

### 2.5 Update FubuniChat

- Add auth check before chat
- Show AuthModal when unauthenticated
- Add Authorization header to API calls

## Step 3: Backend Integration

### 3.1 Install Dependencies

```bash
cd backend
pip install python-jose[cryptography] httpx
```

### 3.2 Update Settings

Add `AUTH_SERVICE_URL` to settings.

### 3.3 Create Middleware

Create `backend/app/middleware/auth.py` with JWT validation.

### 3.4 Protect Endpoints

Add `require_auth` dependency to chat endpoints.

## Step 4: Testing

### 4.1 Manual Testing Flow

1. Start all services:
   ```bash
   # Terminal 1: Auth service
   cd auth-service && npm run dev

   # Terminal 2: FastAPI
   cd backend && uvicorn app.main:app --reload

   # Terminal 3: Frontend
   npm start
   ```

2. Test signup flow:
   - Click chat bubble
   - Fill signup form
   - Verify account created and logged in

3. Test login flow:
   - Sign out
   - Log in with same credentials
   - Verify session persists on refresh

4. Test Google OAuth:
   - Click "Sign in with Google"
   - Complete OAuth flow
   - Verify logged in

5. Test protected endpoints:
   - Without auth: should get 401
   - With auth: should work normally

### 4.2 API Testing

```bash
# Test auth service health
curl http://localhost:4000/health

# Test signup
curl -X POST http://localhost:4000/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123"}'

# Test protected endpoint without auth (should fail)
curl http://localhost:8000/api/chat/sessions

# Test protected endpoint with auth
curl http://localhost:8000/api/chat/sessions \
  -H "Authorization: Bearer <session-token>"
```

## Environment Variables Summary

### Auth Service (.env)
| Variable | Required | Description |
|----------|----------|-------------|
| AUTH_PORT | Yes | Server port (default: 4000) |
| NEON_DATABASE_URL | Yes | PostgreSQL connection string |
| BETTER_AUTH_SECRET | Yes | 32+ char secret for encryption |
| BETTER_AUTH_URL | Yes | Auth service public URL |
| GOOGLE_CLIENT_ID | For OAuth | Google OAuth client ID |
| GOOGLE_CLIENT_SECRET | For OAuth | Google OAuth client secret |

### FastAPI Backend (.env addition)
| Variable | Required | Description |
|----------|----------|-------------|
| AUTH_SERVICE_URL | Yes | Auth service URL for validation |

## Common Issues

### CORS Errors
Ensure all origins are listed in both auth-service and FastAPI CORS config.

### Session Not Persisting
Check cookie settings: `sameSite`, `secure`, `httpOnly`.

### Google OAuth Redirect Error
Verify redirect URI matches exactly in Google Console.

### Database Connection Failed
Check NEON_DATABASE_URL format and SSL mode.

## Next Steps After Setup

1. Run `/sp.tasks` to generate implementation tasks
2. Implement in order: auth-service → frontend → backend
3. Test each component independently
4. Integration test the full flow
