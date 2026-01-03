# Replit Deployment Checklist

## Pre-deployment
- [ ] Verify all environment variables are properly set in Replit Secrets
- [ ] Confirm BETTER_AUTH_SECRET is at least 32 characters
- [ ] Ensure DATABASE_URL is correctly configured
- [ ] Verify CORS_ORIGINS includes Replit URL and frontend domains
- [ ] Check that BETTER_AUTH_URL matches the Replit deployment URL

## Deployment Steps
- [ ] Import repository from GitHub to Replit
- [ ] Verify .replit configuration is correct
- [ ] Install dependencies with `npm install`
- [ ] Run database migrations if needed: `npm run db:migrate`
- [ ] Start the service using the "Run" button
- [ ] Check the console for any startup errors

## Post-deployment Verification
- [ ] Service is accessible at Replit URL
- [ ] Health check endpoint responds: `/health`
- [ ] Wake-up endpoint responds: `/wake-up`
- [ ] Authentication endpoints are accessible
- [ ] CORS headers are properly configured
- [ ] Database connection is established
- [ ] Frontend can communicate with auth service

## Testing
- [ ] Test user registration flow
- [ ] Test user login flow
- [ ] Verify session management works
- [ ] Test JWT token generation endpoint
- [ ] Verify logout functionality

## Replit-specific Considerations
- [ ] Service handles sleep/wake cycles properly
- [ ] Response times are acceptable (under 10 seconds)
- [ ] No sensitive data is logged to console