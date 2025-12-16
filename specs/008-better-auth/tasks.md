# Tasks: Better Auth Integration

**Input**: Design documents from `/specs/008-better-auth/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: No explicit test-first approach requested. E2E tests included in final phase.

**Organization**: Tasks grouped by user story priority (P1 → P2) to enable independent implementation.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US5)
- Include exact file paths in descriptions

## Path Conventions

- **Auth Service (NEW)**: `auth-service/src/`
- **Frontend (EXISTING)**: `src/` (Docusaurus)
- **Backend (EXISTING)**: `backend/app/`

---

## Phase 1: Setup (Auth Service Infrastructure)

**Purpose**: Create auth-service project structure and dependencies

- [x] T001 Create auth-service directory structure at `auth-service/`
- [x] T002 Initialize Node.js project with package.json at `auth-service/package.json`
- [x] T003 [P] Create TypeScript config at `auth-service/tsconfig.json`
- [x] T004 [P] Create environment template at `auth-service/.env.example`
- [x] T005 Install dependencies: better-auth, express, pg, cors, dotenv

---

## Phase 2: Foundational (Core Auth Service)

**Purpose**: Core auth infrastructure that MUST be complete before ANY user story

**⚠️ CRITICAL**: No user story work can begin until auth service is functional

- [x] T006 Create PostgreSQL connection pool at `auth-service/src/db.ts`
- [x] T007 Configure Better Auth with email/password at `auth-service/src/auth.ts`
- [x] T008 Create Express server with health endpoint at `auth-service/src/index.ts`
- [x] T009 Run Better Auth database migration to create ba_* tables
- [x] T010 Verify auth service starts on port 4000 and /health responds
- [x] T011 [P] Add better-auth dependency to frontend at `package.json`
- [x] T012 [P] Add python-jose and httpx to backend at `backend/requirements.txt`

**Checkpoint**: Auth service running, database tables created, dependencies installed

---

## Phase 3: User Story 1 - New User Registration (Priority: P1) 🎯 MVP

**Goal**: New visitors can create accounts with email/password and start chatting

**Independent Test**: Complete signup flow → verify user can send first message

### Implementation for User Story 1

- [x] T013 [US1] Create auth client with signup function at `src/lib/auth-client.ts`
- [x] T014 [US1] Create AuthProvider context at `src/components/Auth/AuthProvider.tsx`
- [x] T015 [US1] Create signup form UI in AuthModal at `src/components/Auth/AuthModal.tsx`
- [x] T016 [US1] Wrap app with AuthProvider at `src/theme/Root.tsx`
- [x] T017 [US1] Add auth check to FubuniChat - show modal when unauthenticated at `src/components/FubuniChat/FubuniChat.tsx`
- [x] T018 [US1] Create auth middleware with require_auth dependency at `backend/app/middleware/auth.py`
- [x] T019 [US1] Add auth_service_url setting at `backend/app/config/settings.py`
- [x] T020 [US1] Protect POST /api/chat endpoint with require_auth at `backend/app/api/chat.py`
- [x] T021 [US1] Add Authorization header to chat API calls in FubuniChat at `src/components/FubuniChat/FubuniChat.tsx`
- [x] T022 [US1] Store authenticated user_id in chat_sessions at `backend/app/api/chat.py`

**Checkpoint**: New user can signup → login automatically → send chat message → message saved with user_id

---

## Phase 4: User Story 2 - Returning User Login (Priority: P1)

**Goal**: Returning users can login and see their chat history

**Independent Test**: Login with existing account → verify session persists on refresh → see past chats

### Implementation for User Story 2

- [x] T023 [US2] Add login form to AuthModal (toggle login/signup) at `src/components/Auth/AuthModal.tsx`
- [x] T024 [US2] Implement session persistence check on page load at `src/components/Auth/AuthProvider.tsx`
- [x] T025 [US2] Protect GET /api/chat/sessions - filter by user_id at `backend/app/api/chat.py`
- [x] T026 [US2] Protect GET /api/chat/history/{id} - validate ownership at `backend/app/api/chat.py`
- [ ] T027 [US2] Add chat history loading to FubuniChat for authenticated users at `src/components/FubuniChat/FubuniChat.tsx`
- [x] T028 [US2] Handle session expiry with re-auth prompt at `src/components/Auth/AuthProvider.tsx`

**Checkpoint**: User can login → refresh page stays logged in → see only their chat history

---

## Phase 5: User Story 5 - Protected Chat Access (Priority: P1)

**Goal**: Unauthenticated requests are properly rejected with clear feedback

**Independent Test**: Send unauthenticated API request → verify 401 response

### Implementation for User Story 5

- [x] T029 [US5] Protect POST /api/chat/stream with require_auth at `backend/app/api/chat.py`
- [x] T030 [US5] Add get_current_user optional dependency for public endpoints at `backend/app/middleware/auth.py`
- [x] T031 [US5] Ensure /health and /api/rag/* remain public (no auth) at `backend/app/main.py`
- [x] T032 [US5] Handle 401 responses in frontend - show auth modal at `src/components/FubuniChat/FubuniChat.tsx`
- [x] T033 [US5] Add error state UI for authentication failures at `src/components/Auth/AuthModal.tsx`

**Checkpoint**: All protected endpoints return 401 when unauthenticated; public endpoints work without auth

---

## Phase 6: User Story 3 - Google Sign-In (Priority: P2)

**Goal**: Users can sign in with Google OAuth for convenience

**Independent Test**: Complete Google OAuth flow → land in authenticated chat state

### Implementation for User Story 3

- [x] T034 [US3] Add Google OAuth provider to Better Auth config at `auth-service/src/auth.ts`
- [x] T035 [US3] Add Google credentials to environment template at `auth-service/.env.example`
- [x] T036 [US3] Add "Sign in with Google" button to AuthModal at `src/components/Auth/AuthModal.tsx`
- [x] T037 [US3] Implement Google sign-in handler in auth client at `src/lib/auth-client.ts`
- [x] T038 [US3] Handle OAuth callback redirect at `src/components/Auth/AuthProvider.tsx`
- [x] T039 [US3] Configure account linking for same email at `auth-service/src/auth.ts`

**Checkpoint**: User can click Google button → complete OAuth → land authenticated in chat

---

## Phase 7: User Story 4 - User Sign Out (Priority: P2)

**Goal**: Users can sign out to protect privacy or switch accounts

**Independent Test**: Sign out → verify protected resources inaccessible → auth modal shown

### Implementation for User Story 4

- [x] T040 [US4] Add signOut function to auth client at `src/lib/auth-client.ts`
- [x] T041 [US4] Add sign out button to chat header at `src/components/FubuniChat/FubuniChat.tsx`
- [x] T042 [US4] Clear session state on sign out at `src/components/Auth/AuthProvider.tsx`
- [x] T043 [US4] Show auth modal after sign out at `src/components/FubuniChat/FubuniChat.tsx`

**Checkpoint**: User can sign out → see auth modal → can't access chat until re-login

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: E2E testing, deployment prep, documentation

- [x] T044 [P] Create Dockerfile for auth-service at `auth-service/Dockerfile`
- [x] T045 [P] Create README for auth-service at `auth-service/README.md`
- [x] T046 [P] Update CORS config for production URLs at `auth-service/src/index.ts`
- [x] T047 [P] Update CORS config in FastAPI for auth-service at `backend/app/main.py`
- [ ] T048 Create E2E test for full auth flow at `tests/e2e/auth.spec.ts`
- [ ] T049 Validate quickstart.md setup instructions work
- [ ] T050 Update environment documentation with all new variables

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **US1 (Phase 3)**: Depends on Foundational - Core MVP
- **US2 (Phase 4)**: Depends on US1 (needs auth client and modal)
- **US5 (Phase 5)**: Depends on US1 (needs middleware)
- **US3 (Phase 6)**: Depends on US1 (needs auth UI foundation)
- **US4 (Phase 7)**: Depends on US1 (needs auth state)
- **Polish (Phase 8)**: Depends on all user stories complete

### User Story Dependencies

```
Foundational (Phase 2)
        │
        ▼
┌───────────────────────────────────────────────────┐
│           User Story 1 (Registration) P1         │
│              Core MVP - Must Complete First       │
└───────────────────────────────────────────────────┘
        │
        ├──────────────┬──────────────┬─────────────┐
        ▼              ▼              ▼             ▼
   US2 (Login)    US5 (Protected)  US3 (Google)  US4 (SignOut)
      P1              P1              P2            P2
```

### Parallel Opportunities

**Within Setup (Phase 1)**:
- T003 (tsconfig) + T004 (env.example) can run in parallel

**Within Foundational (Phase 2)**:
- T011 (frontend deps) + T012 (backend deps) can run in parallel

**After US1 Complete**:
- US2, US5, US3, US4 can proceed in parallel if team capacity allows
- Recommended: Complete US2 + US5 (both P1) before US3 + US4 (both P2)

**Within Polish (Phase 8)**:
- T044, T045, T046, T047 can all run in parallel

---

## Parallel Example: After Foundational

```bash
# If team has 2 developers after US1:

# Developer A: User Story 2 (Login)
T023 → T024 → T025 → T026 → T027 → T028

# Developer B: User Story 5 (Protected Access)
T029 → T030 → T031 → T032 → T033
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T012)
3. Complete Phase 3: User Story 1 (T013-T022)
4. **STOP and VALIDATE**: Test signup → chat flow works
5. Deploy if MVP is sufficient

### Full Feature (All User Stories)

1. Complete MVP (Phases 1-3)
2. Add US2 (Login) + US5 (Protected) - both P1
3. Add US3 (Google) + US4 (SignOut) - both P2
4. Complete Polish (Phase 8)
5. Full E2E validation

### Incremental Delivery

| Milestone | User Stories | Value Delivered |
|-----------|--------------|-----------------|
| MVP | US1 | New users can register and chat |
| Login | US1 + US2 | Returning users can access history |
| Secure | US1 + US2 + US5 | All endpoints properly protected |
| Social | + US3 | Google OAuth for convenience |
| Complete | + US4 | Full auth lifecycle |

---

## Task Summary

| Phase | Tasks | Parallel Tasks | Story |
|-------|-------|----------------|-------|
| Setup | 5 | 2 | - |
| Foundational | 7 | 2 | - |
| US1 Registration | 10 | 0 | US1 |
| US2 Login | 6 | 0 | US2 |
| US5 Protected | 5 | 0 | US5 |
| US3 Google | 6 | 0 | US3 |
| US4 SignOut | 4 | 0 | US4 |
| Polish | 7 | 4 | - |
| **Total** | **50** | **8** | - |

---

## Notes

- [P] tasks can run in parallel (different files)
- [Story] label maps task to user story for traceability
- Each user story is independently testable after completion
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Public endpoints (/health, /api/rag/*) must remain accessible throughout
