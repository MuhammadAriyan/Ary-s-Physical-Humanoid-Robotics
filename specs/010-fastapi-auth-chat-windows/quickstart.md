# Quickstart: Authenticated Chat with Session Windows

**Feature**: 010-fastapi-auth-chat-windows
**Date**: 2025-12-21

## Prerequisites

- Python 3.11+
- Node.js 18+
- Neon PostgreSQL account (branch: Ary-auth-accounts)
- Google Cloud Console project (for OAuth)

## Environment Setup

### Backend (.env)

```bash
# Database
NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/neondb?sslmode=require

# JWT Configuration
JWT_PRIVATE_KEY="-----BEGIN RSA PRIVATE KEY-----\n...\n-----END RSA PRIVATE KEY-----"
JWT_ISSUER=https://fubuni.com
JWT_AUDIENCE=fubuni-chat
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=15
JWT_REFRESH_TOKEN_EXPIRE_DAYS=7

# Google OAuth
GOOGLE_CLIENT_ID=xxx.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=xxx

# Frontend URL (for OAuth redirects)
FRONTEND_URL=http://localhost:3000

# Existing settings
OPENROUTER_API_KEY=sk-or-v1-xxx
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
```

### Generate RSA Keys

```bash
# Generate private key
openssl genrsa -out private.pem 2048

# Extract public key (optional, derived from private)
openssl rsa -in private.pem -pubout -out public.pem

# Convert to single-line format for .env
cat private.pem | tr '\n' '\\n' | sed 's/\\n$//'
```

## Quick Start

### 1. Install Backend Dependencies

```bash
cd backend
pip install pyjwt[crypto] python-jose authlib bcrypt
```

### 2. Run Database Migration

```bash
# Connect to Neon and run migration
psql $NEON_DATABASE_URL < specs/010-fastapi-auth-chat-windows/data-model.md
# Or run the migration SQL from data-model.md manually
```

### 3. Start Backend

```bash
cd backend
uvicorn app.main:app --reload --port 8000
```

### 4. Start Frontend

```bash
npm start
# Opens at http://localhost:3000
```

### 5. Test Auth Flow

```bash
# Register
curl -X POST http://localhost:8000/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123","name":"Test User"}'

# Login
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123"}'

# Get profile (use access_token from login response)
curl http://localhost:8000/api/auth/me \
  -H "Authorization: Bearer <access_token>"

# Get JWKS
curl http://localhost:8000/.well-known/jwks.json
```

### 6. Test Chat Flow

```bash
# List sessions (empty initially)
curl http://localhost:8000/api/chat/sessions \
  -H "Authorization: Bearer <access_token>"

# Send message (creates session)
curl -X POST http://localhost:8000/api/chat \
  -H "Authorization: Bearer <access_token>" \
  -H "Content-Type: application/json" \
  -d '{"message":"What are servo motors?"}'

# List sessions (now has one)
curl http://localhost:8000/api/chat/sessions \
  -H "Authorization: Bearer <access_token>"
```

## Development Workflow

### Backend Development

1. Edit files in `backend/app/`
2. FastAPI auto-reloads on save
3. Test endpoints with curl or Swagger UI at http://localhost:8000/docs

### Frontend Development

1. Edit files in `src/`
2. Docusaurus hot-reloads on save
3. Check browser console for errors

### Testing Auth

1. Open http://localhost:3000/chat
2. See auth modal (not logged in)
3. Register or login
4. See chat interface with session sidebar
5. Send messages, see sidebar update
6. Refresh page, verify session persists

## Common Issues

### "Invalid token" on API calls
- Check access token hasn't expired (15 min)
- Verify JWT_ISSUER and JWT_AUDIENCE match between backend and token

### "Email already registered"
- User exists - use login instead
- Check ba_user table for existing accounts

### Google OAuth redirect fails
- Verify GOOGLE_CLIENT_ID and GOOGLE_CLIENT_SECRET
- Check redirect URI matches in Google Console
- Ensure FRONTEND_URL is correct

### Sessions not persisting
- Check localStorage has access_token and refresh_token
- Verify NEON_DATABASE_URL is correct
- Check browser console for 401 errors

## File Structure

```
backend/
├── app/
│   ├── api/
│   │   ├── auth.py          # Auth endpoints (NEW)
│   │   └── chat.py          # Chat endpoints (MODIFIED)
│   ├── config/
│   │   ├── keys.py          # JWT key manager (NEW)
│   │   └── settings.py      # Settings (MODIFIED)
│   ├── middleware/
│   │   └── auth.py          # Auth middleware (MODIFIED)
│   └── models/
│       └── auth.py          # Auth models (NEW)

src/
├── lib/
│   └── auth.ts              # Auth client (NEW)
├── components/
│   ├── Auth/
│   │   ├── AuthProvider.tsx # (MODIFIED)
│   │   └── AuthModal.tsx    # (MODIFIED)
│   └── Chat/
│       └── SessionSidebar.tsx # (NEW)
└── pages/
    └── chat.tsx             # (MODIFIED)
```

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Implement backend auth endpoints
3. Implement frontend auth client
4. Add session sidebar to chat page
5. Test full flow
