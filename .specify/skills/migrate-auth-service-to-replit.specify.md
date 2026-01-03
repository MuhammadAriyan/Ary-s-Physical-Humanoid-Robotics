# Migrate Auth Service from Koyeb to Replit

## Objective
Migrate the Better Auth + Express authentication service from Koyeb hosting to Replit hosting platform while maintaining all functionality and ensuring proper CORS configuration.

## Current State
- Auth service running on Koyeb using Better Auth and Express
- Service handles authentication for the application
- CORS configured for frontend applications
- Environment variables for secrets and configuration

## Target State
- Auth service running on Replit (Node.js repl)
- All Better Auth functionality preserved
- Proper CORS configuration for frontend applications
- Environment variables securely configured
- Service accessible via Replit URL

## Requirements
- Maintain all existing auth functionality
- Preserve user sessions and data
- Ensure CORS works properly with frontend applications
- Secure handling of environment variables
- Zero downtime during migration (if possible)
- Proper error handling and logging

## Constraints
- Must work within Replit's free tier limitations
- Service may experience sleep/wake cycles on free tier
- Replit URL will be different from Koyeb URL
- Some performance limitations compared to dedicated hosting

## Success Criteria
- Auth service runs successfully on Replit
- All authentication endpoints work properly
- CORS allows frontend applications to connect
- Environment variables are properly configured
- Frontend applications can connect to new service URL
- No data loss during migration