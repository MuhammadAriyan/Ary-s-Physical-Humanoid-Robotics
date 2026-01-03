# Feature Specification: Consolidate Better Auth service into FastAPI backend

**Feature Branch**: `011-auth-consolidation`
**Created**: 2026-01-03
**Status**: Draft
**Input**: User description: "Prompt

 Create a detailed feature specification for consolidating the Node.js Better Auth service into the Python FastAPI backend. The specification should include:

 1. Feature Requirements:
   - Replace Node.js auth service with Python FastAPI auth endpoints
   - Maintain API compatibility with existing frontend
   - Support same authentication methods (email/password, social login)
   - Generate compatible JWT tokens for backend API access
   - Maintain user session management
 2. Technical Requirements:
   - Use python-jose for JWT handling
   - Use passlib for password hashing
   - Implement database models matching Better Auth schema
   - Maintain CORS configuration for GitHub Pages
   - Ensure cookie security settings match production requirements
 3. API Contracts:
   - /auth/sign-in - user login with email/password
   - /auth/sign-up - user registration
   - /auth/session - get current session
   - /auth/token - generate JWT for backend access
   - /auth/sign-out - logout functionality
 4. Database Schema:
   - User table with email, password hash, name, verification status
   - Session table with user reference and expiration
   - Account table for social login integration
 5. Security Considerations:
   - Password hashing with bcrypt
   - Secure JWT signing with shared secret
   - Proper session management and cleanup
   - Input validation and sanitization"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - User Registration (Priority: P1)

As a new user, I want to register for an account so that I can access personalized features.

**Why this priority**: This is the foundational capability that enables all other functionality. Without registration, no other user journeys are possible.

**Independent Test**: Can be fully tested by creating a new account with email/password and verifying the account is created in the database. Delivers the core value of enabling new users to join the system.

**Acceptance Scenarios**:

1. **Given** I am a new user on the registration page, **When** I enter valid email, password, and name and submit, **Then** I should receive confirmation of successful registration and be able to log in.
2. **Given** I am a new user with invalid input, **When** I attempt to register, **Then** I should receive appropriate validation error messages.

---

### User Story 2 - User Authentication (Priority: P1)

As a registered user, I want to log in to my account so that I can access my data and personalized features.

**Why this priority**: This is the essential capability for existing users to access the system. It's the second most critical after registration.

**Independent Test**: Can be fully tested by logging in with valid credentials and receiving a valid session/JWT token. Delivers the core value of allowing users to access protected resources.

**Acceptance Scenarios**:

1. **Given** I am a registered user with valid credentials, **When** I submit my email and password, **Then** I should be authenticated and receive a valid session.
2. **Given** I am a user with invalid credentials, **When** I attempt to log in, **Then** I should receive an appropriate error message.

---

### User Story 3 - JWT Token Generation (Priority: P2)

As an authenticated user, I want my JWT token to be generated so that I can access protected API endpoints in the backend service.

**Why this priority**: This enables the backend API integration that is critical for the overall system functionality, but requires authentication to work first.

**Independent Test**: Can be fully tested by authenticating a user and requesting a JWT token, then verifying the token can be used for backend API calls. Delivers the value of enabling secure API access.

**Acceptance Scenarios**:

1. **Given** I am an authenticated user, **When** I request a JWT token, **Then** I should receive a valid JWT token that can be used for backend API calls.
2. **Given** I am not authenticated, **When** I request a JWT token, **Then** I should receive an unauthorized error.

---

### User Story 4 - Social Login (Priority: P2)

As a user, I want to sign in using my Google or GitHub account so that I don't need to create a new password.

**Why this priority**: This provides a convenient authentication method that improves user experience, but email/password authentication is the foundation.

**Independent Test**: Can be fully tested by using Google or GitHub OAuth to authenticate and creating/linking an account. Delivers the value of providing alternative authentication methods.

**Acceptance Scenarios**:

1. **Given** I have a Google or GitHub account, **When** I click a Google or GitHub login button and complete the OAuth flow, **Then** I should be authenticated and have an account created or linked.

---

### User Story 5 - User Session Management (Priority: P1)

As an authenticated user, I want my session to be maintained so that I don't need to log in repeatedly during my visit.

**Why this priority**: This is essential for good user experience and security, allowing users to stay logged in while maintaining security.

**Independent Test**: Can be fully tested by logging in and maintaining the session across multiple requests, then logging out. Delivers the value of convenient access with proper security.

**Acceptance Scenarios**:

1. **Given** I am logged in, **When** I make multiple requests over time, **Then** I should remain authenticated until session expires.
2. **Given** I am logged in, **When** I explicitly log out, **Then** my session should be terminated.

---

### Edge Cases

- What happens when a user tries to register with an email that already exists?
- How does the system handle expired JWT tokens?
- What happens when the database is unavailable during authentication?
- How does the system handle malformed JWT tokens?
- What happens when a user attempts to authenticate with an unverified email (if email verification is required)?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST provide `/auth/sign-up` endpoint for user registration with email, password, and name
- **FR-002**: System MUST provide `/auth/sign-in` endpoint for user authentication with email and password
- **FR-003**: System MUST validate user credentials using bcrypt for password hashing
- **FR-004**: System MUST provide `/auth/token` endpoint to generate JWT tokens for backend API access
- **FR-005**: System MUST provide `/auth/session` endpoint to retrieve current session information
- **FR-006**: System MUST provide `/auth/sign-out` endpoint to terminate user sessions
- **FR-007**: System MUST implement social login functionality for Google and GitHub OAuth providers
- **FR-008**: System MUST store user data in PostgreSQL database with proper schema
- **FR-009**: System MUST validate all input data and return appropriate error messages
- **FR-010**: System MUST implement proper CORS configuration for GitHub Pages domain access
- **FR-011**: System MUST generate JWT tokens with proper signing using shared secret
- **FR-012**: System MUST maintain session state with appropriate expiration times
- **FR-013**: System MUST set authentication cookies with HttpOnly, Secure, and SameSite=Lax attributes
- **FR-014**: System MUST set JWT access token expiration to 15 minutes and refresh token expiration to 1 day

### Key Entities *(include if feature involves data)*

- **User**: Represents a registered user with email, password hash, name, verification status, and timestamps
- **Session**: Represents an active user session with user reference, token, and expiration time
- **Account**: Represents a social login provider account linked to a user with provider details

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Users can complete account registration with email/password in under 10 seconds
- **SC-002**: Users can authenticate with email/password in under 2 seconds
- **SC-003**: JWT tokens are generated and validated successfully for backend API access
- **SC-004**: 99% of authentication requests succeed under normal load conditions
- **SC-005**: User sessions are properly maintained across page refreshes and browser tabs
- **SC-006**: Social login providers successfully authenticate users in 95% of attempts
- **SC-007**: Passwords are properly hashed and stored securely using industry standards
- **SC-008**: CORS requests from GitHub Pages domain are properly handled with appropriate headers
- **SC-009**: System can handle at least 100 concurrent authentication requests without degradation
- **SC-010**: All authentication endpoints return appropriate error messages for invalid inputs

## Clarifications

### Session 2026-01-03

- Q: Which social login providers should be supported? → A: Google and GitHub OAuth only
- Q: What security attributes should be set on authentication cookies? → A: HttpOnly, Secure, SameSite=Lax
- Q: What should be the JWT token expiration times? → A: 15 minutes for access token, 1 day for refresh token
