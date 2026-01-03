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