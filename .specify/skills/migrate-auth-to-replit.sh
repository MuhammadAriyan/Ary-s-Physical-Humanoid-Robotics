#!/bin/bash
# Skill to migrate auth service from Koyeb to Replit

echo "Starting migration of auth service from Koyeb to Replit..."

# Run the specification first
echo "Creating specification for migration..."
cat > /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/specs/migrate-auth-service-to-replit/spec.md << 'EOF'
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
EOF

# Run the plan
echo "Creating implementation plan..."
cat > /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/specs/migrate-auth-service-to-replit/plan.md << 'EOF'
# Migration Plan: Auth Service from Koyeb to Replit

## Objective
Create a detailed implementation plan to migrate the Better Auth + Express authentication service from Koyeb to Replit.

## Phase 1: Preparation
### 1.1 Project Setup
- Create new Replit account/workspace if needed
- Create a new Node.js repl named appropriately for the auth service
- Set up the basic project structure matching the current Koyeb deployment

### 1.2 Code Migration
- Copy all source code from current Koyeb deployment
- Ensure package.json has proper start script: `"start": "node src/index.js"`
- Update server code to listen on Replit's environment port
- Verify all dependencies are properly listed in package.json

### 1.3 Configuration Review
- Document all current environment variables used in Koyeb
- Identify any Koyeb-specific configurations that need adjustment
- Review Better Auth configuration for Replit compatibility

## Phase 2: Implementation
### 2.1 Server Code Adaptation
- Modify the main server file to use Replit's PORT environment variable
- Ensure the server listens on '0.0.0.0' for external connections
- Add proper error handling for Replit environment

### 2.2 Better Auth Configuration
- Update Better Auth baseURL to Replit URL format
- Configure CORS origins to include frontend applications
- Ensure session configuration is appropriate for Replit environment
- Test authentication endpoints

### 2.3 Environment Variables Setup
- Set up all required environment variables in Replit's secrets manager
- Include Better Auth secrets, database URLs, and OAuth provider credentials
- Verify all variables are properly encrypted

## Phase 3: Testing
### 3.1 Local Testing
- Test the application locally before deployment
- Verify all authentication endpoints work properly
- Test user registration, login, and session management

### 3.2 Replit Testing
- Deploy to Replit and test the live service
- Verify CORS configuration works with frontend applications
- Test authentication flows end-to-end

### 3.3 Frontend Integration
- Update frontend applications to use new Replit URL
- Test authentication from frontend applications
- Verify all auth-related functionality works as expected

## Phase 4: Deployment and Validation
### 4.1 Final Deployment
- Deploy final working version to Replit
- Document the new Replit URL for the auth service
- Set up any necessary monitoring or logging

### 4.2 Validation
- Confirm all auth endpoints are accessible
- Verify CORS allows proper frontend communication
- Test with actual frontend applications
- Validate that user sessions work correctly

## Risk Mitigation
- Maintain backup of current Koyeb deployment during migration
- Test thoroughly before updating frontend applications
- Plan for potential downtime during switch-over
- Have rollback plan if issues arise

## Success Criteria
- Auth service runs successfully on Replit
- All authentication functionality preserved
- CORS configured properly for frontend applications
- Frontend applications connect successfully to new service
- No data loss during migration
- Service performs adequately within Replit constraints
EOF

echo "Specification and plan created for migrating auth service to Replit."
echo "Files created:"
echo "- specs/migrate-auth-service-to-replit/spec.md"
echo "- specs/migrate-auth-service-to-replit/plan.md"