# Deploy Better Auth to Fly.io

## Prerequisites
- Fly.io account (free, no credit card required for free tier)
- Neon database URL ready

## Step 1: Install Fly CLI

```bash
# Linux/Mac
curl -L https://fly.io/install.sh | sh

# Add to PATH (if needed)
export FLYCTL_INSTALL="$HOME/.fly"
export PATH="$FLYCTL_INSTALL/bin:$PATH"
```

## Step 2: Login to Fly.io

```bash
fly auth login
```

This opens your browser to create/login to your free account.

## Step 3: Deploy from auth-service directory

```bash
cd auth-service

# Launch the app (uses fly.toml config)
fly launch --no-deploy

# It will ask:
# - App name: press Enter to use "fubuni-auth" (from fly.toml)
# - Region: choose closest to you (or press Enter for default)
# - Postgres database: say NO (you're using Neon)
# - Redis: say NO
```

## Step 4: Set Environment Secrets

```bash
# Required secrets
fly secrets set BETTER_AUTH_SECRET="$(openssl rand -base64 32)"
fly secrets set JWT_SECRET="$(openssl rand -base64 32)"
fly secrets set NEON_DATABASE_URL="postgresql://user:pass@host/db?sslmode=require"

# Update this after deployment with your actual URL
fly secrets set BETTER_AUTH_URL="https://fubuni-auth.fly.dev"

# Add your frontend domain(s)
fly secrets set CORS_ORIGINS="https://your-frontend.vercel.app,https://your-domain.com"

# Optional: Google OAuth
fly secrets set GOOGLE_CLIENT_ID="your-client-id"
fly secrets set GOOGLE_CLIENT_SECRET="your-client-secret"
```

## Step 5: Deploy

```bash
fly deploy
```

That's it! Your auth service will be live at: `https://fubuni-auth.fly.dev`

## Step 6: Update Frontend

Update your frontend to use the new auth URL:

```typescript
// In your frontend auth config
export const authClient = createAuthClient({
  baseURL: "https://fubuni-auth.fly.dev"
})
```

## Verify Deployment

```bash
# Check app status
fly status

# View logs
fly logs

# Check health endpoint
curl https://fubuni-auth.fly.dev/health
```

## Important Notes

### Cookie Configuration
Your Better Auth cookies will work across domains because:
- ✅ HTTPS is enforced (`force_https = true`)
- ✅ Always-on (`auto_stop_machines = false`)
- ✅ CORS properly configured via `CORS_ORIGINS`

### Free Tier
- 3 shared VMs (256MB RAM each)
- 160GB outbound data/month
- Always-on (no cold starts)
- Your auth service uses 1 VM = FREE ✅

### Updating Environment Variables

```bash
# View current secrets
fly secrets list

# Update a secret
fly secrets set CORS_ORIGINS="https://new-domain.com"

# Unset a secret
fly secrets unset SOME_SECRET
```

### Monitoring

```bash
# Real-time logs
fly logs

# SSH into the instance
fly ssh console

# Restart the app
fly apps restart fubuni-auth
```

## Troubleshooting

### Issue: Cookies not working
1. Check CORS_ORIGINS includes your frontend domain
2. Verify BETTER_AUTH_URL matches your fly.dev URL
3. Ensure frontend uses HTTPS

### Issue: Database connection fails
1. Check NEON_DATABASE_URL is correct
2. Verify `?sslmode=require` is in the connection string
3. Check Neon database allows external connections

### Issue: App not starting
```bash
fly logs  # Check error messages
fly ssh console  # SSH into instance
```

## Cost Monitoring

```bash
# Check your usage
fly billing

# View app metrics
fly dashboard
```

Your setup is **completely free** as long as you stay within:
- 1 VM running (256MB)
- 160GB/month bandwidth

For a small auth service, this is more than enough! 🚀
