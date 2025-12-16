# Research: Better Auth Integration

**Feature**: 008-better-auth
**Date**: 2025-12-17
**Purpose**: Resolve technical unknowns and document architectural decisions

## Research Topics

### 1. Better Auth Framework Selection

**Decision**: Use Better Auth as the authentication framework

**Rationale**:
- Framework-agnostic TypeScript authentication library
- Built-in support for email/password and 40+ OAuth providers including Google
- Native PostgreSQL support via `pg` adapter
- React client library with hooks (`useSession`, `signIn`, `signUp`)
- Table prefix support (`ba_`) to avoid conflicts with existing schema
- Session and JWT token support out of the box
- Active maintenance and comprehensive documentation

**Alternatives Considered**:
- **Auth.js (NextAuth)**: Tightly coupled to Next.js, not ideal for Docusaurus
- **Lucia Auth**: Good but less mature plugin ecosystem
- **Custom JWT implementation**: More control but significant development effort
- **Firebase Auth**: Vendor lock-in, adds external dependency

---

### 2. Microservice Architecture vs Monolith

**Decision**: Separate Node.js auth service alongside FastAPI backend

**Rationale**:
- Better Auth is TypeScript-native; running in Python would require significant adaptation
- Separation of concerns: auth service handles identity, FastAPI handles business logic
- Independent scaling and deployment
- FastAPI can validate JWT tokens without needing full auth logic
- Both services share the same Neon PostgreSQL database

**Alternatives Considered**:
- **Python-native auth (FastAPI-Users)**: Would integrate better but lacks Better Auth's features
- **Single Node.js backend**: Would require rewriting existing FastAPI chat logic
- **Auth0/Clerk SaaS**: Adds external dependency and cost

---

### 3. JWT Token Validation Strategy

**Decision**: FastAPI validates tokens by calling auth-service session endpoint

**Rationale**:
- Auth-service is the single source of truth for session validity
- Avoids secret sharing between services
- Supports session revocation (logout invalidates token immediately)
- Simple HTTP call with caching potential

**Alternatives Considered**:
- **Shared JWT secret**: Faster validation but requires secret distribution
- **Public key verification**: More complex setup, Better Auth defaults to session-based
- **Token introspection endpoint**: Same as chosen approach but standardized

---

### 4. Frontend SSR Compatibility (Docusaurus)

**Decision**: Use Better Auth React client with SSR-safe patterns

**Rationale**:
- Better Auth React client handles SSR via `typeof window` checks
- Auth state stored in cookies (SSR-accessible) and localStorage (client-side)
- Docusaurus builds static pages; auth is client-side only after hydration
- Auth modal renders only on client (no flash of unauthenticated content)

**Implementation Pattern**:
```typescript
// SSR-safe auth client initialization
const getAuthUrl = () => {
  if (typeof window === 'undefined') return '';
  return window.location.hostname === 'localhost'
    ? 'http://localhost:4000'
    : 'https://production-auth-url';
};
```

---

### 5. Database Schema Strategy

**Decision**: Use `ba_` table prefix for Better Auth tables

**Rationale**:
- Better Auth supports configurable table prefixes via `advanced.database.tablePrefix`
- Existing tables (`chat_sessions`, `chat_messages`) remain unchanged
- Clear separation: `ba_user`, `ba_session`, `ba_account`, `ba_verification`
- Single database connection shared, no additional infrastructure

**Tables Created**:
| Table | Purpose |
|-------|---------|
| `ba_user` | User accounts (id, email, name, email_verified, image) |
| `ba_session` | Active sessions (token, user_id, expires_at) |
| `ba_account` | Auth methods linked to user (password hash, OAuth tokens) |
| `ba_verification` | Email verification tokens |

---

### 6. Google OAuth Configuration

**Decision**: Configure Google OAuth via Better Auth socialProviders plugin

**Rationale**:
- Better Auth has built-in Google provider support
- Requires Google Cloud Console OAuth 2.0 credentials
- Callback URL: `{AUTH_SERVICE_URL}/api/auth/callback/google`
- Scopes: email, profile (minimal required)

**Prerequisites**:
1. Google Cloud Console project
2. OAuth 2.0 Client ID (Web application type)
3. Authorized redirect URIs configured
4. Client ID and Secret in environment variables

---

### 7. Session Persistence Strategy

**Decision**: Cookie-based sessions with 7-day expiration

**Rationale**:
- Better Auth default session management is cookie-based
- Cookies persist across browser sessions (unlike localStorage for tokens)
- HttpOnly cookies prevent XSS token theft
- 7-day expiration balances security and UX
- Session refresh on activity extends validity

**Configuration**:
```typescript
session: {
  expiresIn: 60 * 60 * 24 * 7,  // 7 days in seconds
  updateAge: 60 * 60 * 24,      // Refresh daily
  cookieOptions: {
    sameSite: "lax",
    secure: process.env.NODE_ENV === "production",
    httpOnly: true
  }
}
```

---

### 8. CORS Configuration

**Decision**: Configure matching CORS origins across all services

**Rationale**:
- Frontend (Docusaurus) needs to call both auth-service and FastAPI
- Credentials must be included for cookie-based auth
- Origins must match exactly (no wildcards with credentials)

**Origins**:
- Development: `http://localhost:3000`, `http://localhost:3001`
- Production: `https://muhammadariyan.github.io`

---

### 9. Deployment Strategy (Hugging Face Spaces)

**Decision**: Deploy auth-service as separate Hugging Face Space

**Rationale**:
- HF Spaces supports Node.js via Docker
- Can share same Neon PostgreSQL as FastAPI Space
- Free tier available for auth service (low traffic)
- Both services get HTTPS automatically

**Configuration**:
- Auth service: `Dockerfile` with Node.js 20
- Port: 7860 (HF Spaces default)
- Health check: `GET /health`

---

## Constitution Compliance Analysis

### Potential Violations

| Principle | Status | Justification |
|-----------|--------|---------------|
| Backend MUST be Python 3.11 | VIOLATION | Auth service is Node.js - justified because Better Auth is TypeScript-native; FastAPI remains Python for all business logic |
| Static deployability | COMPLIANT | Auth service is separate deployment; frontend remains static |
| No RAG/vector DB | COMPLIANT | Authentication is unrelated to RAG |
| OpenAI-compatible providers | COMPLIANT | Auth is separate from AI agent; agent still uses OpenAI SDK |

### Justification for Node.js Auth Service

The constitution's Python requirement applies to the **AI agent backend**, not auxiliary services. The Better Auth integration:
1. Does not modify the existing FastAPI AI backend
2. Provides a specialized service for a specialized purpose (auth)
3. Could theoretically be replaced with any auth provider without affecting the agent
4. Follows the principle of using the best tool for each job

---

## Resolved Unknowns Summary

| Unknown | Resolution |
|---------|------------|
| Auth framework choice | Better Auth (TypeScript, PostgreSQL, React client) |
| Service architecture | Microservice (Node.js auth + Python FastAPI) |
| Token validation | Session endpoint call from FastAPI to auth-service |
| SSR compatibility | Client-side only auth with SSR-safe patterns |
| Database schema | `ba_` prefixed tables in shared Neon PostgreSQL |
| Google OAuth | Better Auth socialProviders with GCP credentials |
| Session persistence | Cookie-based, 7-day expiration |
| CORS | Explicit origin list across all services |
| Deployment | Separate HF Spaces for auth-service |
