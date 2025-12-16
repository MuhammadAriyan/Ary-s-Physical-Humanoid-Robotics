# Implementation Plan: Better Auth Integration

**Branch**: `008-better-auth` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/008-better-auth/spec.md`

## Summary

Integrate Better Auth authentication system into Fubuni Chat application using a microservice architecture. A separate Node.js auth service handles user registration, login (email/password + Google OAuth), and session management. The existing FastAPI backend validates sessions and protects chat endpoints. The React frontend displays an auth modal requiring login before chat access.

## Technical Context

**Language/Version**: Node.js 20 (auth-service), Python 3.11 (FastAPI), TypeScript 5.6 (React)
**Primary Dependencies**: Better Auth, Express, FastAPI, React 19, Docusaurus 3
**Storage**: Neon PostgreSQL (shared, `ba_` prefix for auth tables)
**Testing**: Jest (auth-service), pytest (FastAPI), Playwright (E2E)
**Target Platform**: Web (Docusaurus static site + API services)
**Project Type**: Web application (multi-service)
**Performance Goals**: Auth operations < 1s, session validation < 100ms
**Constraints**: No breaking changes to existing functionality, SSR-compatible
**Scale/Scope**: Single-tenant, supports concurrent users on existing infrastructure

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Agent named "Fubuni" | N/A | Auth feature doesn't affect agent identity |
| OpenAI-compatible providers | COMPLIANT | Auth is separate from AI; agent unchanged |
| Backend Python 3.11 + OpenAI SDK | **VIOLATION** | Auth service is Node.js (see justification) |
| Frontend matches Docusaurus theme | COMPLIANT | Auth UI uses existing styling |
| Static deployable, no Node runtime | **VIOLATION** | Auth service requires Node (see justification) |
| No RAG/vector DB initially | COMPLIANT | Auth is unrelated to RAG |
| Future extensibility | COMPLIANT | Auth designed for extensibility |

### Constitution Violation Justifications

See [Complexity Tracking](#complexity-tracking) below for detailed justifications.

## Project Structure

### Documentation (this feature)

```text
specs/008-better-auth/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Technical research decisions
├── data-model.md        # Entity definitions
├── quickstart.md        # Setup guide
├── contracts/           # API specifications
│   ├── auth-service-api.yaml
│   └── fastapi-chat-api.yaml
├── checklists/
│   └── requirements.md  # Quality checklist
└── tasks.md             # Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
# Auth Service (NEW)
auth-service/
├── package.json
├── tsconfig.json
├── .env.example
├── Dockerfile
├── README.md
└── src/
    ├── index.ts         # Express server entry point
    ├── auth.ts          # Better Auth configuration
    └── db.ts            # PostgreSQL connection

# Frontend (EXISTING - modifications)
src/
├── lib/
│   └── auth-client.ts   # NEW: Better Auth React client
├── components/
│   ├── Auth/            # NEW: Auth components
│   │   ├── AuthProvider.tsx
│   │   └── AuthModal.tsx
│   └── FubuniChat/
│       └── FubuniChat.tsx  # MODIFIED: Add auth checks
└── theme/
    └── Root.tsx            # MODIFIED: Add AuthProvider

# Backend (EXISTING - modifications)
backend/
├── app/
│   ├── middleware/
│   │   └── auth.py      # NEW: JWT validation
│   ├── config/
│   │   └── settings.py  # MODIFIED: Add auth_service_url
│   └── api/
│       └── chat.py      # MODIFIED: Add require_auth
└── requirements.txt     # MODIFIED: Add python-jose, httpx
```

**Structure Decision**: Multi-service web application with isolated auth microservice. Auth service is self-contained in `auth-service/` folder, minimizing impact on existing codebase.

## Complexity Tracking

> **Constitution violations justified below**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Node.js auth service (not Python) | Better Auth is TypeScript-native with no Python port; provides superior auth features (40+ providers, plugins) | Python alternatives (FastAPI-Users) lack Better Auth's features and ecosystem; would require building OAuth from scratch |
| Auth service requires Node runtime | Better Auth requires Node.js; authentication is a specialized concern separate from AI agent | Embedding auth in FastAPI would require custom implementation; no Python library matches Better Auth's capabilities |

**Mitigation**: Auth service is isolated and can be replaced without affecting core AI agent functionality. FastAPI remains Python for all business logic.

## Implementation Phases

### Phase 1: Auth Service Foundation

**Goal**: Standalone auth service with email/password authentication

**Files to Create**:
- `auth-service/package.json`
- `auth-service/tsconfig.json`
- `auth-service/.env.example`
- `auth-service/src/db.ts`
- `auth-service/src/auth.ts`
- `auth-service/src/index.ts`

**Acceptance Criteria**:
- Auth service starts on port 4000
- Health endpoint responds at `/health`
- Database migration creates `ba_*` tables
- Sign-up creates user and returns session
- Sign-in validates credentials and returns session

---

### Phase 2: Google OAuth Integration

**Goal**: Add Google sign-in to auth service

**Files to Modify**:
- `auth-service/src/auth.ts` (add socialProviders)
- `auth-service/.env.example` (add Google credentials)

**Acceptance Criteria**:
- Google OAuth redirect works
- Callback creates/links user account
- Session returned after OAuth completion

**Prerequisites**: Google Cloud Console OAuth credentials

---

### Phase 3: Frontend Auth Client

**Goal**: React client for authentication state

**Files to Create**:
- `src/lib/auth-client.ts`
- `src/components/Auth/AuthProvider.tsx`

**Files to Modify**:
- `package.json` (add better-auth)
- `src/theme/Root.tsx` (wrap with AuthProvider)

**Acceptance Criteria**:
- `useAuth` hook provides auth state
- Session persists across page refreshes
- Sign-out clears session

---

### Phase 4: Auth UI Components

**Goal**: Login/signup modal UI

**Files to Create**:
- `src/components/Auth/AuthModal.tsx`

**Files to Modify**:
- `src/components/FubuniChat/FubuniChat.tsx`

**Acceptance Criteria**:
- Modal displays login/signup forms
- Toggle between login and signup modes
- Google sign-in button works
- Error messages display correctly
- Loading states during auth operations

---

### Phase 5: Backend Auth Middleware

**Goal**: FastAPI validates auth tokens

**Files to Create**:
- `backend/app/middleware/auth.py`

**Files to Modify**:
- `backend/requirements.txt` (add python-jose, httpx)
- `backend/app/config/settings.py` (add auth_service_url)

**Acceptance Criteria**:
- `require_auth` dependency validates tokens
- `get_current_user` returns user info
- Invalid tokens return 401
- Public endpoints remain accessible

---

### Phase 6: Protected Chat Endpoints

**Goal**: Chat endpoints require authentication

**Files to Modify**:
- `backend/app/api/chat.py`

**Acceptance Criteria**:
- POST /api/chat requires auth
- POST /api/chat/stream requires auth
- GET /api/chat/sessions returns only user's sessions
- GET /api/chat/history/{id} validates session ownership
- user_id stored as authenticated user ID

---

### Phase 7: Integration & Testing

**Goal**: End-to-end auth flow works

**Files to Create**:
- `auth-service/Dockerfile`
- `tests/e2e/auth.spec.ts` (Playwright)

**Acceptance Criteria**:
- Full signup → chat → signout flow works
- Google OAuth flow works
- Session persists across refreshes
- Protected endpoints reject unauthenticated requests
- Public endpoints work without auth

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Google OAuth setup complexity | Medium | Medium | Detailed quickstart guide; fallback to email/password only |
| CORS issues between services | High | Low | Document exact configuration; test early |
| Session cookie issues in production | Medium | High | Test HTTPS cookie behavior before production deploy |
| Better Auth breaking changes | Low | Medium | Pin dependency versions; review changelog before updates |

## Dependencies

### External
- Google Cloud Console project (for OAuth)
- Neon PostgreSQL access (existing)

### Internal
- Existing chat functionality must continue working
- Frontend styling must match Docusaurus theme

## Definition of Done

- [ ] All Phase acceptance criteria met
- [ ] No regression in existing functionality
- [ ] All public endpoints remain accessible
- [ ] E2E tests pass for auth flows
- [ ] Documentation updated (quickstart.md)
- [ ] Environment variables documented

## Related Artifacts

- **Spec**: [spec.md](./spec.md)
- **Research**: [research.md](./research.md)
- **Data Model**: [data-model.md](./data-model.md)
- **Quickstart**: [quickstart.md](./quickstart.md)
- **Contracts**: [contracts/](./contracts/)
