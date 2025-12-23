# Feature Specification: Authenticated Chat with Session Windows

**Feature Branch**: `010-fastapi-auth-chat-windows`
**Created**: 2025-12-21
**Status**: Draft
**Input**: User description: "Migrate Express.js auth to FastAPI with JWT/JWKS, require authentication for chat, add chat session windows (conversations) that persist, allow users to create new chat windows and switch between previous ones, auto-generate titles from first message"

## Overview

This feature transforms the chat system from an anonymous-access model to a fully authenticated experience. Users must sign in (email/password or Google OAuth) before chatting. Each conversation is stored as a "chat window" that persists, allowing users to return to previous conversations and start new ones. The authentication backend migrates from a separate Express.js service to the existing FastAPI backend using industry-standard JWT/JWKS patterns.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Registration and First Chat (Priority: P1)

A new user visits the chat page, creates an account with email and password, and has their first conversation with Fubuni. The conversation is automatically saved as a chat window with an auto-generated title.

**Why this priority**: This is the core user acquisition flow - without registration and first chat working, the feature has zero value.

**Independent Test**: Can be fully tested by registering a new account and sending one message. Delivers immediate value as user can chat and their conversation is saved.

**Acceptance Scenarios**:

1. **Given** a user visits the chat page without being logged in, **When** they view the page, **Then** they see a sign-in/sign-up form instead of the chat interface
2. **Given** a user is on the registration form, **When** they enter a valid email, password (minimum 8 characters), and optional name, **Then** their account is created and they are automatically signed in
3. **Given** a newly registered user, **When** they send their first message, **Then** a new chat session is created with the title auto-generated from the first message (truncated to 50 characters)
4. **Given** a user has sent messages, **When** they refresh the page or return later, **Then** they see their previous conversation in the sidebar

---

### User Story 2 - Returning User Sign-In and Chat History (Priority: P1)

An existing user signs in with their email and password, sees their previous chat windows in a sidebar, and can continue any previous conversation or start a new one.

**Why this priority**: Equally critical as registration - returning users need to access their history to derive ongoing value.

**Independent Test**: Can be tested by logging in with existing credentials and verifying chat history appears in sidebar.

**Acceptance Scenarios**:

1. **Given** a user with an existing account visits the chat page, **When** they enter correct credentials and submit, **Then** they are signed in and see the chat interface with their previous sessions in a sidebar
2. **Given** a signed-in user with multiple chat sessions, **When** they view the sidebar, **Then** they see a list of their chat windows ordered by most recently active, with titles and relative timestamps (e.g., "2 hours ago")
3. **Given** a signed-in user viewing the sidebar, **When** they click on a previous chat session, **Then** the full message history for that session loads in the chat panel
4. **Given** a signed-in user, **When** they click "New Chat" button, **Then** a new empty chat session is created and becomes active

---

### User Story 3 - Google OAuth Sign-In (Priority: P2)

A user chooses to sign in with their Google account instead of creating a new email/password account. They are authenticated and can chat immediately.

**Why this priority**: Important for user convenience and conversion, but email/password provides baseline functionality.

**Independent Test**: Can be tested by clicking "Sign in with Google", completing OAuth flow, and verifying access to chat.

**Acceptance Scenarios**:

1. **Given** a user on the sign-in form, **When** they click "Sign in with Google", **Then** they are redirected to Google's authentication page
2. **Given** a user completes Google authentication, **When** they are redirected back, **Then** their account is created (if new) or linked (if email matches existing account), and they are signed in
3. **Given** a user who previously signed up with email but now uses Google OAuth with the same email, **When** they complete Google sign-in, **Then** the accounts are linked and they see their existing chat history

---

### User Story 4 - Session Persistence Across Browser Sessions (Priority: P2)

A user signs in, closes their browser, and returns later. They should still be signed in (within a reasonable time period) without re-entering credentials.

**Why this priority**: Critical for user experience but depends on core auth working first.

**Independent Test**: Can be tested by signing in, closing browser, reopening, and verifying still authenticated.

**Acceptance Scenarios**:

1. **Given** a user signs in successfully, **When** they close the browser and return within 7 days, **Then** they are still signed in and see their chat history
2. **Given** a user's session has expired (after 7 days of inactivity), **When** they visit the chat page, **Then** they are prompted to sign in again
3. **Given** a user is signed in, **When** their short-term access expires (15 minutes), **Then** the system automatically refreshes their session without interrupting their chat

---

### User Story 5 - Delete Chat Window (Priority: P3)

A user wants to remove a chat conversation they no longer need. They can delete it from the sidebar.

**Why this priority**: Nice-to-have for user control, but not essential for core functionality.

**Independent Test**: Can be tested by creating a chat session, deleting it, and verifying it no longer appears.

**Acceptance Scenarios**:

1. **Given** a signed-in user with chat sessions in the sidebar, **When** they click the delete action on a session, **Then** they see a confirmation prompt
2. **Given** a user confirms deletion, **When** the action completes, **Then** the session and all its messages are permanently removed from the sidebar and database
3. **Given** a user deletes their currently active session, **When** deletion completes, **Then** they see an empty chat state or are switched to another session

---

### User Story 6 - Sign Out (Priority: P3)

A user wants to sign out of their account, especially on shared devices.

**Why this priority**: Important for security but not blocking core chat functionality.

**Independent Test**: Can be tested by signing out and verifying subsequent visits require re-authentication.

**Acceptance Scenarios**:

1. **Given** a signed-in user, **When** they click the sign-out button, **Then** they are signed out and see the sign-in form
2. **Given** a user has signed out, **When** they try to access the chat page, **Then** they must sign in again to access their history

---

### Edge Cases

- What happens when a user tries to register with an email that already exists? → Show clear error message and suggest sign-in
- What happens when a user enters wrong password? → Show error message, allow retry, implement rate limiting after 5 failures
- What happens when Google OAuth is cancelled mid-flow? → Return user to sign-in form with no error
- What happens when a user tries to access another user's chat session? → Return 403 Forbidden, do not reveal session exists
- What happens when the chat page loads but the backend is unavailable? → Show friendly error message with retry option
- What happens when a user's refresh token expires while they're mid-conversation? → Prompt to re-authenticate without losing typed message

## Requirements *(mandatory)*

### Functional Requirements

#### Authentication
- **FR-001**: System MUST allow users to create accounts with email and password
- **FR-002**: System MUST validate that passwords are at least 8 characters long
- **FR-003**: System MUST validate email format and uniqueness during registration
- **FR-004**: System MUST allow users to sign in with email and password
- **FR-005**: System MUST support Google OAuth 2.0 as an alternative sign-in method
- **FR-006**: System MUST automatically link accounts when Google OAuth email matches existing email/password account
- **FR-007**: System MUST maintain user sessions that persist across browser restarts
- **FR-008**: System MUST automatically refresh sessions without user intervention when short-term credentials expire
- **FR-009**: System MUST allow users to sign out, clearing their session
- **FR-010**: System MUST protect authentication tokens from common web vulnerabilities (XSS, CSRF)

#### Chat Session Management
- **FR-011**: System MUST require authentication before allowing access to chat functionality
- **FR-012**: System MUST display a sidebar showing all of the user's chat sessions
- **FR-013**: System MUST order chat sessions by most recent activity (newest first)
- **FR-014**: System MUST display session title and relative timestamp in the sidebar
- **FR-015**: System MUST auto-generate session titles from the first message (truncated to 50 characters)
- **FR-016**: System MUST allow users to create new chat sessions
- **FR-017**: System MUST allow users to switch between chat sessions by clicking in the sidebar
- **FR-018**: System MUST load and display full message history when a session is selected
- **FR-019**: System MUST allow users to delete chat sessions they own
- **FR-020**: System MUST enforce user isolation - users can only access their own sessions

#### Data Persistence
- **FR-021**: System MUST persist all chat messages within sessions
- **FR-022**: System MUST update session's "last active" timestamp when new messages are added
- **FR-023**: System MUST preserve existing user accounts from the previous authentication system
- **FR-024**: System MUST cascade delete all messages when a session is deleted

### Key Entities

- **User**: Represents a registered user with email, optional name, optional profile image, and email verification status. A user can have multiple authentication methods (email/password, Google OAuth) and owns multiple chat sessions.

- **Chat Session (Window)**: Represents a conversation thread owned by a single user. Contains a title (auto-generated or manual), creation timestamp, last activity timestamp, and active status. A session contains multiple messages.

- **Chat Message**: Represents a single message within a session. Has a sender (user or assistant), content text, sequence number for ordering, and timestamp.

- **Authentication Method**: Represents how a user authenticates (email/password with hashed password, or Google OAuth with OAuth tokens). A user may have multiple methods linked.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete registration and send their first chat message in under 2 minutes
- **SC-002**: Users can sign in and access their chat history in under 10 seconds
- **SC-003**: Chat session sidebar loads and displays all sessions within 500 milliseconds
- **SC-004**: 100% of users can only access their own chat sessions (no unauthorized access)
- **SC-005**: Existing users from the previous authentication system can sign in without re-registering
- **SC-006**: System supports at least 100 concurrent authenticated users without degradation
- **SC-007**: Session tokens remain valid for 7 days, reducing re-authentication friction
- **SC-008**: Google OAuth sign-in completes the full flow in under 30 seconds
- **SC-009**: Users can switch between chat sessions and see message history load within 1 second
- **SC-010**: Zero data loss - all messages persist correctly across sessions and browser restarts

## Assumptions

1. **Password Hashing Compatibility**: The existing user database uses bcrypt for password hashing, which is industry standard and will be compatible with the new backend
2. **Database Availability**: The Neon PostgreSQL database branch (Ary-auth-accounts) contains the existing user data and is accessible
3. **Google OAuth Configuration**: Google Cloud Console project with OAuth credentials will be configured separately (client ID and secret)
4. **Frontend Framework**: The chat page is built with React in a Docusaurus site, supporting standard React patterns
5. **Session Storage**: Browser localStorage is available and acceptable for storing authentication tokens
6. **Mobile Responsiveness**: On mobile devices, the sidebar may be collapsed or hidden to prioritize the chat interface
7. **RAG System Independence**: The existing RAG (retrieval-augmented generation) system and its data will not be modified by this feature

## Constraints

1. **Database Reuse**: Must use existing `ba_user` and `ba_account` tables - no user migration or data loss acceptable
2. **Backend Consolidation**: Authentication must be handled by the existing FastAPI backend (port 8000), not a separate service
3. **Stateless Authentication**: Token validation must not require database lookups for every request (performance requirement)
4. **No Anonymous Chat**: After this feature, anonymous/unauthenticated chat will no longer be supported

## Out of Scope

- Email verification flow (not currently required)
- Password reset/forgot password functionality
- Two-factor authentication (2FA)
- Multiple device session management (viewing/revoking other sessions)
- Chat session sharing between users
- Chat export functionality
- Manual session title editing (auto-generated only in this version)
- Admin panel for user management
