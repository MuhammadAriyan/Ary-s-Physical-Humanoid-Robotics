# Tasks: Authenticated Chat with Session Windows

**Input**: Design documents from `/specs/010-fastapi-auth-chat-windows/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: No automated tests requested. Manual testing per quickstart.md.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

**IMPORTANT**: Better Auth library is MANDATORY. Keep existing auth-service (Express.js at port 4000) and auth-client.ts using better-auth/react.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (FastAPI)
- **Frontend**: `src/` (Docusaurus React)
- **Auth Service**: Existing Express.js Better Auth service (port 4000)
- **Specs**: `specs/010-fastapi-auth-chat-windows/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify existing Better Auth setup and add session management infrastructure

- [x] T001 Verify Better Auth auth-service is configured (existing at port 4000)
- [x] T002 Verify auth-client.ts uses better-auth/react (existing src/lib/auth-client.ts)
- [x] T003 Verify backend middleware calls Better Auth session endpoint (existing backend/app/middleware/auth.py)

---

## Phase 2: Foundational (Session Management Infrastructure)

**Purpose**: Add chat session sidebar infrastructure while keeping Better Auth

**CRITICAL**: Better Auth remains the authentication provider - no replacement

### Backend Core

- [x] T004 Add session listing endpoint improvements to backend/app/api/chat.py (order by last_interaction DESC)
- [x] T005 Add POST /api/chat/sessions endpoint to create new empty session in backend/app/api/chat.py
- [x] T006 Add DELETE /api/chat/sessions/{id} endpoint for session deletion in backend/app/api/chat.py
- [x] T007 Update GET /api/chat/sessions endpoint to return proper session list format with message_count

### Frontend Core (Keep Better Auth)

- [x] T008 [P] Export getSession function from src/lib/auth-client.ts for getting Better Auth session token
- [x] T009 [P] Create session API helpers in src/lib/chat-sessions.ts (listSessions, createSession, deleteSession, getSessionMessages)

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - New User Registration and First Chat (Priority: P1) MVP

**Goal**: New users can create an account via Better Auth, sign in automatically, and have their first chat saved with auto-generated title

**Independent Test**: Register new account via Better Auth, send message, verify session appears in sidebar with auto-generated title

### Implementation for User Story 1

#### Backend - Chat Session Creation (Better Auth validates tokens)

- [x] T010 [US1] Update POST /api/chat in backend/app/api/chat.py to use require_auth (already calls Better Auth)
- [x] T011 [US1] Verify auto-title generation in backend/app/api/chat.py (extract first 50 chars from first message) - ALREADY EXISTS
- [x] T012 [US1] Update POST /api/chat/stream in backend/app/api/chat.py to use require_auth consistently

#### Frontend - Auth Gate (Using Better Auth)

- [x] T013 [US1] Import useAuth from AuthProvider in src/pages/chat.tsx
- [x] T014 [US1] Add auth gate to src/pages/chat.tsx (show AuthModal when not authenticated using Better Auth)
- [x] T015 [US1] Add Authorization header to chat API calls in src/pages/chat.tsx (Bearer token from Better Auth session)

#### Frontend - Session Sidebar (Basic)

- [x] T016 [US1] Create src/components/Chat/SessionSidebar.tsx component (list sessions, highlight active)
- [x] T017 [US1] Create src/components/Chat/SessionSidebar.module.css (240px width, left position)
- [x] T018 [US1] Update src/pages/chat.tsx to include SessionSidebar component
- [x] T019 [US1] Update src/pages/chat.module.css with sidebar grid layout (240px 1fr)

**Checkpoint**: User Story 1 complete - new users can register via Better Auth, chat, and see their session in sidebar

---

## Phase 4: User Story 2 - Returning User Sign-In and Chat History (Priority: P1)

**Goal**: Existing users can sign in via Better Auth and see their previous chat sessions in the sidebar, switch between them, and start new chats

**Independent Test**: Login via Better Auth with existing credentials, verify all previous sessions appear, click session to load history

### Implementation for User Story 2

#### Backend - Session Listing (Better Auth validates tokens)

- [x] T020 [US2] Verify GET /api/chat/sessions returns sessions ordered by last_interaction DESC - ALREADY EXISTS
- [x] T021 [US2] Verify GET /api/chat/history/{session_id} returns session with messages - ALREADY EXISTS
- [x] T022 [US2] Add POST /api/chat/sessions endpoint in backend/app/api/chat.py (create new empty session)

#### Frontend - Session Display (Using Better Auth)

- [x] T023 [US2] Add session fetching to SessionSidebar.tsx (GET /api/chat/sessions on mount with auth token)
- [x] T024 [US2] Add session switching to SessionSidebar.tsx (onClick loads session with messages)
- [x] T025 [US2] Add "New Chat" button to SessionSidebar.tsx (POST /api/chat/sessions)
- [x] T026 [US2] Update src/pages/chat.tsx to load messages when session is selected
- [x] T027 [US2] Add relative timestamps to SessionSidebar.tsx (e.g., "2 hours ago")

**Checkpoint**: User Story 2 complete - returning users can login via Better Auth, see history, switch sessions

---

## Phase 5: User Story 3 - Google OAuth Sign-In (Priority: P2)

**Goal**: Users can sign in with Google account via Better Auth (already supported)

**Independent Test**: Click "Sign in with Google" in Better Auth modal, complete OAuth flow, verify access to chat

### Implementation for User Story 3

- [x] T028 [US3] Verify Google OAuth is configured in Better Auth auth-service
- [x] T029 [US3] Verify signInWithGoogle in src/lib/auth-client.ts calls Better Auth social sign-in - ALREADY EXISTS
- [x] T030 [US3] Test Google OAuth flow end-to-end (manual testing required)

**Checkpoint**: User Story 3 complete - Google OAuth via Better Auth working (minimal changes needed)

---

## Phase 6: User Story 4 - Session Persistence Across Browser Sessions (Priority: P2)

**Goal**: Users remain signed in after closing browser via Better Auth session management

**Independent Test**: Sign in via Better Auth, close browser, reopen, verify still authenticated

### Implementation for User Story 4

- [x] T031 [US4] Verify Better Auth handles session persistence (cookies/localStorage)
- [x] T032 [US4] Test session restoration on page load via AuthProvider.tsx
- [x] T033 [US4] Handle session expiry gracefully in chat page (show re-auth modal)

**Checkpoint**: User Story 4 complete - Better Auth sessions persist across browser restarts

---

## Phase 7: User Story 5 - Delete Chat Window (Priority: P3)

**Goal**: Users can delete chat sessions they no longer need

**Independent Test**: Create session, delete it, verify removed from sidebar and database

### Implementation for User Story 5

#### Backend - Delete Session

- [x] T034 [US5] Add DELETE /api/chat/sessions/{id} endpoint in backend/app/api/chat.py (verify ownership, cascade delete messages)

#### Frontend - Delete UI

- [x] T035 [US5] Add delete button/icon to session items in SessionSidebar.tsx
- [x] T036 [US5] Add confirmation dialog for delete in SessionSidebar.tsx
- [x] T037 [US5] Handle active session deletion in src/pages/chat.tsx (switch to another session or empty state)

**Checkpoint**: User Story 5 complete - session deletion working

---

## Phase 8: User Story 6 - Sign Out (Priority: P3)

**Goal**: Users can sign out via Better Auth to secure their account

**Independent Test**: Sign out via Better Auth, verify must re-authenticate to access chat

### Implementation for User Story 6

- [x] T038 [US6] Verify signOut in src/lib/auth-client.ts calls Better Auth - ALREADY EXISTS
- [x] T039 [US6] Add sign out button to chat page header in src/pages/chat.tsx
- [x] T040 [US6] Clear chat state on sign out and show auth modal

**Checkpoint**: User Story 6 complete - sign out via Better Auth working

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T041 [P] Add mobile responsive styles to SessionSidebar.module.css (hide sidebar < 768px, hamburger menu)
- [x] T042 [P] Add loading states to SessionSidebar.tsx (skeleton loaders)
- [x] T043 [P] Add error handling for API failures (user-friendly messages)
- [x] T044 Add edge case handling for typed message preservation during re-auth prompt
- [ ] T045 Run quickstart.md validation (manual end-to-end test)

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup - Verify Existing Better Auth) ✓ COMPLETE
    │
    ▼
Phase 2 (Foundational - Session Management)
    │
    ├──► Phase 3 (US1 - P1) ──► Phase 4 (US2 - P1)
    │                                    │
    │                                    ├──► Phase 5 (US3 - P2) ← Minimal (Better Auth handles OAuth)
    │                                    │
    │                                    ├──► Phase 6 (US4 - P2) ← Minimal (Better Auth handles persistence)
    │                                    │
    │                                    ├──► Phase 7 (US5 - P3)
    │                                    │
    │                                    └──► Phase 8 (US6 - P3) ← Minimal (Better Auth signOut exists)
    │
    └──► Phase 9 (Polish) ─── After all desired user stories
```

### User Story Dependencies

| User Story | Depends On | Can Start After |
|------------|------------|-----------------|
| US1 (P1) | Foundational only | Phase 2 complete |
| US2 (P1) | US1 (shares sidebar) | Phase 3 complete |
| US3 (P2) | US1/US2 | Phase 4 complete (mostly verification) |
| US4 (P2) | US2 | Phase 4 complete (mostly verification) |
| US5 (P3) | US2 (needs session list UI) | Phase 4 complete |
| US6 (P3) | US2 | Phase 4 complete (mostly verification) |

### Within Each User Story

- Backend endpoints before frontend integration
- Core functionality before UI polish
- Better Auth provides: registration, login, OAuth, session management, signOut

### Parallel Opportunities

**Phase 2 (Foundational)**:
- T008 and T009 can run in parallel (different files)

**Phase 3 (US1)**:
- T016-T019 can run in parallel (sidebar component + CSS)

**Phase 9 (Polish)**:
- T041, T042, T043 all parallelizable

---

## Implementation Strategy

### MVP First (User Story 1 + 2)

1. ✓ Phase 1: Setup (3 tasks) - **ALREADY COMPLETE** (Better Auth exists)
2. Complete Phase 2: Foundational (6 tasks) - Session management helpers
3. Complete Phase 3: User Story 1 (10 tasks) - Auth gate + Sidebar
4. Complete Phase 4: User Story 2 (8 tasks) - Session switching
5. **STOP and VALIDATE**: Full auth flow + chat sessions working
6. Deploy/demo MVP

### Incremental Delivery

1. **MVP (P1 stories)**: Setup + Foundational + US1 + US2 = ~27 tasks
2. **+ Google OAuth (P2)**: US3 = 3 tasks (verification only)
3. **+ Session Persistence (P2)**: US4 = 3 tasks (verification only)
4. **+ Delete Sessions (P3)**: US5 = 4 tasks
5. **+ Sign Out (P3)**: US6 = 3 tasks
6. **Polish**: Phase 9 = 5 tasks

---

## Summary

| Phase | User Story | Priority | Task Count |
|-------|------------|----------|------------|
| 1 | Setup (Better Auth Verification) | - | 3 ✓ |
| 2 | Foundational (Session Management) | - | 6 |
| 3 | US1: Auth Gate + First Chat | P1 | 10 |
| 4 | US2: Session History | P1 | 8 |
| 5 | US3: Google OAuth (Verification) | P2 | 3 |
| 6 | US4: Session Persistence (Verification) | P2 | 3 |
| 7 | US5: Delete Session | P3 | 4 |
| 8 | US6: Sign Out | P3 | 3 |
| 9 | Polish | - | 5 |
| **Total** | | | **45** |

---

## Notes

- **Better Auth is MANDATORY** - all authentication uses existing Better Auth setup
- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story should be independently testable after completion
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- MVP = Phase 1-4 (~27 tasks) delivers core functionality
- Many tasks are "verification only" since Better Auth already handles the functionality
