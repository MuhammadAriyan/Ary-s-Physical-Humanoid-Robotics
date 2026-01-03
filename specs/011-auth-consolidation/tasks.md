# Implementation Tasks: Consolidate Better Auth service into FastAPI backend

**Feature**: 011-auth-consolidation
**Created**: 2026-01-03
**Status**: Ready for implementation

## Phase 1: Auth Module Structure and Basic Models (Days 1-2)

### T001 - Create Auth Module Structure (Day 1)
- [x] T001.1: Create `/backend/app/auth/` directory
- [x] T001.2: Create `auth.py` with main authentication logic
- [x] T001.3: Create `models.py` with database models
- [x] T001.4: Create `schemas.py` with Pydantic schemas
- [x] T001.5: Create `router.py` with FastAPI router definition
- [x] T001.6: Add dependencies to `requirements.txt`

### T002 - Implement User Database Model (Day 1)
- [x] T002.1: Define User model with email (unique, not null)
- [x] T002.2: Add password_hash field with proper constraints
- [x] T002.3: Add name field (not null)
- [x] T002.4: Add email_verified boolean field (default: False)
- [x] T002.5: Add created_at and updated_at timestamp fields
- [x] T002.6: Test model creation and validation

### T003 - Implement Session Database Model (Day 1)
- [x] T003.1: Define Session model with user_id foreign key
- [x] T003.2: Add session_token field (unique, not null)
- [x] T003.3: Add expires_at timestamp field (not null)
- [x] T003.4: Add created_at timestamp field (default: now)
- [x] T003.5: Test session model creation and relationships

### T004 - Implement Account Database Model (Day 1-2)
- [x] T004.1: Define Account model with user_id foreign key
- [x] T004.2: Add provider field (not null) for social providers
- [x] T004.3: Add provider_account_id field (not null)
- [x] T004.4: Add provider_user_id field (not null)
- [x] T004.5: Add created_at timestamp field (default: now)
- [x] T004.6: Test account model and relationships

### T005 - Update Database Initialization (Day 2)
- [x] T005.1: Update `/backend/app/database/init.py` to include auth models
- [x] T005.2: Ensure proper table creation order
- [x] T005.3: Test database initialization with auth tables
- [x] T005.4: Verify no conflicts with existing tables

## Phase 2: Authentication Endpoints (Days 2-3)

### T006 - Implement Sign-Up Endpoint (Day 2)
- [x] T006.1: Create `/auth/sign-up` POST endpoint
- [x] T006.2: Implement request validation schema
- [x] T006.3: Add email and password validation
- [x] T006.4: Implement bcrypt password hashing
- [x] T006.5: Create user in database
- [x] T006.6: Return appropriate success response
- [x] T006.7: Handle duplicate email errors

### T007 - Implement Sign-In Endpoint (Day 2)
- [x] T007.1: Create `/auth/sign-in` POST endpoint
- [x] T007.2: Implement credential validation
- [x] T007.3: Add password verification with bcrypt
- [x] T007.4: Create session record in database
- [x] T007.5: Return user and session information
- [x] T007.6: Handle invalid credentials errors

### T008 - Implement Session Endpoint (Day 2)
- [x] T008.1: Create `/auth/session` GET endpoint
- [x] T008.2: Implement session validation logic
- [x] T008.3: Return current user and session data
- [x] T008.4: Handle unauthenticated requests
- [x] T008.5: Test session retrieval functionality

### T009 - Implement Token Endpoint (Day 3)
- [x] T009.1: Create `/auth/token` GET endpoint
- [x] T009.2: Implement JWT token generation with python-jose
- [x] T009.3: Add user data to token payload
- [x] T009.4: Ensure token structure matches backend expectations
- [x] T009.5: Return JWT token in response
- [x] T009.6: Handle unauthorized token requests

### T010 - Implement Sign-Out Endpoint (Day 3)
- [x] T010.1: Create `/auth/sign-out` POST endpoint
- [x] T010.2: Implement session termination logic
- [x] T010.3: Remove session from database
- [x] T010.4: Return success confirmation
- [x] T010.5: Test sign-out functionality

## Phase 3: JWT Token Implementation (Days 3-4)

### T011 - Set Up JWT Configuration (Day 3)
- [x] T011.1: Install python-jose dependency
- [x] T011.2: Configure JWT_SECRET from environment
- [x] T011.3: Set up JWT algorithm (HS256)
- [x] T011.4: Create JWT utility functions
- [x] T011.5: Test basic JWT creation and verification

### T012 - Implement JWT Token Generation (Day 3)
- [x] T012.1: Create function to generate JWT tokens
- [x] T012.2: Add user_id to token payload
- [x] T012.3: Add email to token payload
- [x] T012.4: Add name to token payload
- [x] T012.5: Add email_verified status to token payload
- [x] T012.6: Set appropriate expiration time (7 days)

### T013 - Ensure JWT Compatibility (Day 3-4)
- [x] T013.1: Match token structure to existing backend expectations
- [x] T013.2: Verify payload fields match Better Auth format
- [x] T013.3: Test JWT with existing backend verification logic
- [x] T013.4: Ensure token signing algorithm compatibility
- [x] T013.5: Validate token expiration handling

### T014 - Implement JWT Validation (Day 4)
- [x] T014.1: Create JWT validation utility function
- [x] T014.2: Add token expiration checking
- [x] T014.3: Implement error handling for invalid tokens
- [x] T014.4: Test token validation with various scenarios
- [x] T014.5: Ensure proper error responses

## Phase 4: Social Login Integration (Days 4-5)

### T015 - Set Up OAuth2 Dependencies (Day 4)
- [x] T015.1: Install requests-oauthlib dependency
- [x] T015.2: Configure OAuth2 provider settings
- [x] T015.3: Set up environment variables for provider credentials
- [x] T015.4: Create OAuth2 utility functions
- [x] T015.5: Test basic OAuth2 configuration

### T016 - Implement Google OAuth (Day 4)
- [x] T016.1: Create `/auth/callback/google` endpoint
- [x] T016.2: Implement Google OAuth2 flow
- [x] T016.3: Handle OAuth2 callback processing
- [x] T016.4: Create or link user account
- [x] T016.5: Create session for authenticated user
- [x] T016.6: Test Google OAuth flow

### T017 - Implement Account Linking (Day 4-5)
- [x] T017.1: Create account linking logic
- [x] T017.2: Handle existing email matching
- [x] T017.3: Create new account if no match
- [x] T017.4: Store provider-specific information
- [x] T017.5: Test account linking functionality

## Phase 5: Integration with Existing Chat Functionality (Days 5-6)

### T018 - Update Main Application (Day 5)
- [x] T018.1: Update `/backend/app/main.py` to include auth router
- [x] T018.2: Ensure proper middleware ordering
- [x] T018.3: Update CORS configuration for auth endpoints
- [x] T018.4: Test integration with existing routes
- [x] T018.5: Verify no conflicts with existing functionality

### T019 - Test Auth-Chat Integration (Day 5)
- [x] T019.1: Test JWT token usage with chat endpoints
- [x] T019.2: Verify authentication middleware works
- [x] T019.3: Test user-specific data access
- [x] T019.4: Validate session-based access controls
- [x] T019.5: Fix any integration issues

### T020 - Update Database Relationships (Day 6)
- [x] T020.1: Update chat models to reference user IDs
- [x] T020.2: Ensure proper foreign key relationships
- [x] T020.3: Test user-specific data isolation
- [x] T020.4: Verify data access controls
- [x] T020.5: Update any necessary indexes

## Phase 6: Frontend Compatibility and Testing (Days 6-7)

### T021 - Verify API Endpoint Compatibility (Day 6)
- [x] T021.1: Test all auth endpoints with existing frontend
- [x] T021.2: Verify request/response format compatibility
- [x] T021.3: Test error response formats
- [x] T021.4: Validate status codes match expectations
- [x] T021.5: Fix any compatibility issues

### T022 - Test JWT Token Compatibility (Day 6)
- [x] T022.1: Verify JWT tokens work with existing backend verification
- [x] T022.2: Test token structure matches backend expectations
- [x] T022.3: Validate token expiration handling
- [x] T022.4: Test token refresh scenarios
- [x] T022.5: Ensure seamless integration with chat sessions

### T023 - Frontend Integration Testing (Day 6-7)
- [x] T023.1: Test auth-client.ts with new endpoints
- [x] T023.2: Verify token refresh and session management
- [x] T023.3: Test cross-origin authentication flows
- [x] T023.4: Validate error handling and user feedback
- [x] T023.5: Perform end-to-end user journey testing

## Phase 7: Security and Validation (Days 7-8)

### T024 - Implement Input Validation (Day 7)
- [x] T024.1: Add comprehensive input validation to all endpoints
- [x] T024.2: Implement sanitization for all user inputs
- [x] T024.3: Add rate limiting to prevent brute force attacks
- [x] T024.4: Validate email format and constraints
- [x] T024.5: Test validation with malicious inputs

### T025 - Security Testing (Day 7-8)
- [x] T025.1: Test for common authentication vulnerabilities
- [x] T025.2: Validate secure session handling
- [x] T025.3: Test JWT token security
- [x] T025.4: Verify proper error message sanitization
- [x] T025.5: Perform security scan of auth endpoints

### T026 - Session Management (Day 8)
- [x] T026.1: Implement proper session cleanup
- [x] T026.2: Add session expiration handling
- [x] T026.3: Test concurrent session management
- [x] T026.4: Validate secure session termination
- [x] T026.5: Test session security across requests

## Phase 8: Deployment Preparation (Days 8)

### T027 - Container Configuration (Day 8)
- [x] T027.1: Update `/backend/Dockerfile` with auth dependencies
- [x] T027.2: Add new requirements to `requirements.txt`
- [x] T027.3: Test container build with auth functionality
- [x] T027.4: Optimize container size and build time
- [x] T027.5: Verify container runs with auth endpoints

### T028 - Environment Configuration (Day 8)
- [x] T028.1: Define production environment variables
- [x] T028.2: Configure JWT_SECRET for production
- [x] T028.3: Set up database connection strings
- [x] T028.4: Configure CORS for GitHub Pages domain
- [x] T028.5: Set up social login provider credentials

## Acceptance Criteria

### Functional Acceptance
- [x] Users can register with email/password and account is created
- [x] Users can log in with valid credentials and receive session
- [x] JWT tokens are generated and compatible with backend verification
- [x] Social login providers authenticate users successfully
- [x] Sessions are properly maintained and terminated
- [x] All error scenarios return appropriate responses

### Performance Acceptance
- [x] Authentication requests complete within 2 seconds
- [x] JWT token generation completes within 100ms
- [x] System handles 100 concurrent authentication requests
- [x] Database operations perform within acceptable timeframes

### Security Acceptance
- [x] Passwords are properly hashed with bcrypt
- [x] JWT tokens are securely signed with shared secret
- [x] Input validation prevents injection attacks
- [x] Rate limiting prevents brute force attacks
- [x] Session management follows security best practices

### Compatibility Acceptance
- [x] Existing frontend works with new auth endpoints
- [x] JWT tokens are compatible with existing backend verification
- [x] CORS configuration allows GitHub Pages access
- [x] All API contracts match existing expectations