# Research: Authenticated Chat with Session Windows

**Feature**: 010-fastapi-auth-chat-windows
**Date**: 2025-12-21
**Status**: Complete

## Research Tasks

### 1. JWT/JWKS Implementation in FastAPI

**Decision**: Use PyJWT with RS256 algorithm for JWT signing and python-jose for JWKS format conversion

**Rationale**:
- PyJWT is the most widely used JWT library in Python with excellent documentation
- RS256 (RSA) provides asymmetric signing - private key signs, public key verifies
- JWKS endpoint allows stateless verification without sharing secrets
- Follows the pattern from Hamza's Better Auth + FastAPI guide (Context7)

**Alternatives Considered**:
- python-jose alone: Less intuitive API for key management
- Authlib JWT: More complex, designed for OAuth providers
- HS256 (symmetric): Requires sharing secret with verifiers, less secure for distributed systems

**Implementation Notes**:
- Generate RSA 2048-bit key pair on startup (or load from env)
- Store private key for signing tokens
- Expose public key via `/.well-known/jwks.json`
- Access token: 15 min expiry, contains user_id, email, name
- Refresh token: 7 days expiry, stored hash in database for revocation

### 2. Password Hashing Compatibility

**Decision**: Use bcrypt (via passlib or bcrypt library) to maintain compatibility with existing Better Auth users

**Rationale**:
- Better Auth uses bcrypt for password hashing by default
- Existing users in `ba_account` table have bcrypt-hashed passwords
- Changing hash algorithm would require password reset for all users

**Alternatives Considered**:
- Argon2 (pwdlib): More secure but incompatible with existing hashes
- scrypt: Also incompatible with existing data
- Migration to Argon2: Would require "rehash on next login" strategy, adds complexity

**Implementation Notes**:
- Use `bcrypt.checkpw()` for password verification
- Use `bcrypt.hashpw()` for new password hashing
- Salt is embedded in bcrypt hash format

### 3. Google OAuth Integration

**Decision**: Use Authlib's Starlette/FastAPI OAuth integration

**Rationale**:
- Authlib is the most mature Python OAuth library
- Native Starlette integration works seamlessly with FastAPI
- Handles PKCE, state validation, token exchange automatically
- Used in production by many FastAPI projects

**Alternatives Considered**:
- httpx-oauth: Less mature, fewer features
- Manual implementation: Error-prone, security risks
- fastapi-users: Full framework, too opinionated for this use case

**Implementation Notes**:
- Register Google OAuth client with client_id, client_secret
- Configure redirect URI: `{backend_url}/api/auth/google/callback`
- Scopes: `openid email profile`
- On callback: extract email, name, picture from userinfo
- Find or create user in ba_user, link in ba_account

### 4. Token Storage Strategy (Frontend)

**Decision**: Store tokens in localStorage with automatic refresh handling

**Rationale**:
- localStorage persists across browser sessions (7-day refresh token usable)
- Simpler than httpOnly cookies for SPA architecture
- XSS mitigation through Content Security Policy (not token storage)
- Follows common SPA patterns (Auth0, Firebase)

**Alternatives Considered**:
- httpOnly cookies: More secure against XSS but complex CORS handling, CSRF required
- sessionStorage: Doesn't persist across tabs/sessions
- Memory only: Lost on refresh, poor UX

**Security Mitigations**:
- Short access token expiry (15 min) limits XSS impact window
- Refresh tokens can be revoked server-side
- HTTPS required in production
- Implement token refresh queue to prevent race conditions

### 5. Session Sidebar UI Pattern

**Decision**: Fixed-width sidebar (240px) on left side, collapsible on mobile

**Rationale**:
- Common pattern in chat applications (ChatGPT, Claude, Slack)
- Left sidebar keeps focus on chat content
- Fixed width maintains consistent layout
- Mobile collapse prevents cramped UI

**Alternatives Considered**:
- Right sidebar: Conflicts with existing docs panel
- Floating panel: Inconsistent with established patterns
- Dropdown selector: Poor discoverability for multiple sessions

**Implementation Notes**:
- Desktop: `grid-template-columns: 240px 1fr` (or `240px 1fr 40%` with docs)
- Mobile (<768px): Hide sidebar, add hamburger menu
- Animate sidebar open/close with CSS transitions

### 6. Database Schema for Refresh Tokens

**Decision**: Create `refresh_tokens` table with token hash, expiry, and revocation flag

**Rationale**:
- Enables server-side token revocation (logout, security incidents)
- Hash storage prevents token theft from database compromise
- Expiry enables automatic cleanup
- User relationship enables "logout all devices" future feature

**Alternatives Considered**:
- Stateless refresh tokens: Cannot revoke without key rotation
- Redis token store: Adds infrastructure dependency
- JWT blacklist: Grows unbounded, performance issues

**Schema**:
```sql
CREATE TABLE refresh_tokens (
  id UUID PRIMARY KEY,
  user_id TEXT REFERENCES ba_user(id) ON DELETE CASCADE,
  token_hash TEXT NOT NULL,
  expires_at TIMESTAMP NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  revoked BOOLEAN DEFAULT FALSE
);
```

### 7. Existing Better Auth Tables Reuse

**Decision**: Reuse `ba_user` and `ba_account` tables as-is, ignore `ba_session` and `ba_verification`

**Rationale**:
- Preserves existing user accounts and passwords
- No migration required
- `ba_session` not needed (JWT is stateless)
- `ba_verification` not needed (email verification disabled)

**Implementation Notes**:
- Query `ba_user` for user lookup by email
- Query `ba_account` with `provider_id='credential'` for password verification
- Query `ba_account` with `provider_id='google'` for OAuth linking
- Create new ba_user + ba_account records for new registrations

## Technology Stack Summary

| Component | Technology | Version |
|-----------|------------|---------|
| Backend Framework | FastAPI | 0.115+ |
| JWT Library | PyJWT | 2.8+ |
| JWKS Formatting | python-jose | 3.3+ |
| OAuth Library | Authlib | 1.3+ |
| Password Hashing | bcrypt | 4.0+ |
| Database ORM | SQLModel | 0.0.14+ |
| Database | Neon PostgreSQL | N/A |
| Frontend Framework | React (Docusaurus) | 18+ |
| TypeScript | TypeScript | 5.0+ |

## Resolved Unknowns

| Unknown | Resolution |
|---------|------------|
| JWT algorithm | RS256 (asymmetric) |
| Password compatibility | bcrypt (same as Better Auth) |
| OAuth library | Authlib |
| Token storage | localStorage |
| Sidebar placement | Left, 240px, collapsible |
| Refresh token storage | Database with hash |
| Existing tables | Reuse ba_user, ba_account |
