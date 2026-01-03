# Fubuni Auth Service

Better Auth authentication service for Fubuni Chat.

## Deployment Options

### Replit Deployment (Recommended for Development)

To deploy this service on Replit:

1. Create a new Repl on [Replit](https://replit.com)
2. Choose "Import from GitHub" and enter the repository URL
3. Select "Node.js" as the environment
4. In the "Secrets" tab, add the following environment variables:
   - `BETTER_AUTH_SECRET`: A secret key (at least 32 characters)
   - `DATABASE_URL`: Your PostgreSQL database connection string
   - `JWT_SECRET`: Secret for JWT token generation
   - `CORS_ORIGINS`: Comma-separated list of allowed origins (include your Replit URL)
5. Update the `.replit` file if needed
6. Click "Run" to start the service
7. The service will be available at `https://your-repl-name.your-username.repl.co`

### Local Development

Follow the instructions below for local development setup.

## Features

- Email/Password authentication
- Google OAuth (optional)
- Session management with PostgreSQL
- Cookie-based sessions with 7-day expiration

## Quick Start

### Prerequisites

- Node.js 20+
- PostgreSQL database (Neon recommended)

### Installation

```bash
npm install
```

### Configuration

Copy the environment template:

```bash
cp .env.example .env
```

Configure your environment variables:

| Variable | Description | Required |
|----------|-------------|----------|
| `AUTH_PORT` | Server port (default: 4000) | No |
| `NODE_ENV` | Environment (development/production) | No |
| `NEON_DATABASE_URL` | PostgreSQL connection string | Yes |
| `BETTER_AUTH_SECRET` | Secret key (min 32 chars) | Yes |
| `BETTER_AUTH_URL` | Auth service base URL | Yes |
| `GOOGLE_CLIENT_ID` | Google OAuth client ID | No |
| `GOOGLE_CLIENT_SECRET` | Google OAuth client secret | No |
| `CORS_ORIGINS` | Comma-separated allowed origins | No |

### Database Migration

Run the Better Auth migration to create tables:

```bash
npm run db:migrate
```

This creates the following tables with `ba_` prefix:
- `ba_user` - User accounts
- `ba_session` - User sessions
- `ba_account` - OAuth accounts
- `ba_verification` - Email verification tokens

### Development

```bash
npm run dev
```

### Production

```bash
npm run build
npm start
```

### Docker

```bash
docker build -t fubuni-auth-service .
docker run -p 4000:4000 --env-file .env fubuni-auth-service
```

## API Endpoints

All auth endpoints are mounted at `/api/auth/*`:

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/auth/sign-up/email` | Register with email/password |
| POST | `/api/auth/sign-in/email` | Sign in with email/password |
| GET | `/api/auth/sign-in/social` | Initiate OAuth flow |
| GET | `/api/auth/callback/*` | OAuth callback |
| GET | `/api/auth/get-session` | Get current session |
| POST | `/api/auth/sign-out` | Sign out |

### Health Check

```bash
curl http://localhost:4000/health
```

## Google OAuth Setup

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create or select a project
3. Navigate to APIs & Services > Credentials
4. Create OAuth 2.0 Client ID
5. Add authorized redirect URI: `http://localhost:4000/api/auth/callback/google`
6. Copy Client ID and Secret to `.env`

## Architecture

```
auth-service/
├── src/
│   ├── index.ts    # Express server
│   ├── auth.ts     # Better Auth configuration
│   └── db.ts       # PostgreSQL connection pool
├── Dockerfile
├── package.json
└── tsconfig.json
```

## Security Notes

- Session tokens are stored in secure, HTTP-only cookies
- CORS is configured for specific origins
- Database connections use SSL
- Secrets should never be committed to version control

## Replit-specific Limitations and Workarounds

### Sleep/Wake Cycles
- Free Replit accounts have services that may sleep after inactivity
- First request after sleep may take longer than usual (cold start)
- Use the `/wake-up` endpoint to keep the service responsive

### Resource Constraints
- Free tier has limited memory and CPU allocation
- Performance may vary based on Replit's current load
- Not suitable for production traffic

### CORS Considerations
- Make sure to include your Replit URL in CORS_ORIGINS
- SameSite cookie attributes are set based on environment detection
- Local development URLs are automatically included

## Troubleshooting

### Service Not Starting
- Check that all required environment variables are set
- Verify database connection string format
- Review console logs for specific error messages

### CORS Issues
- Ensure your frontend domain is included in CORS_ORIGINS
- Check that BETTER_AUTH_URL matches your Replit URL
- Verify trusted origins in Better Auth configuration
