# Data Model: Better Auth Integration

**Feature**: 008-better-auth
**Date**: 2025-12-17

## Entity Relationship Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          BETTER AUTH TABLES (ba_ prefix)                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────┐       ┌──────────────┐       ┌──────────────────┐        │
│  │   ba_user    │       │  ba_session  │       │   ba_account     │        │
│  ├──────────────┤       ├──────────────┤       ├──────────────────┤        │
│  │ id (PK)      │◄──────│ user_id (FK) │       │ id (PK)          │        │
│  │ email        │       │ id (PK)      │       │ user_id (FK)     │───────►│
│  │ name         │       │ token        │       │ account_id       │        │
│  │ email_verified│      │ expires_at   │       │ provider_id      │        │
│  │ image        │       │ ip_address   │       │ access_token     │        │
│  │ created_at   │       │ user_agent   │       │ refresh_token    │        │
│  │ updated_at   │       │ created_at   │       │ password (hashed)│        │
│  └──────────────┘       │ updated_at   │       │ created_at       │        │
│         │               └──────────────┘       │ updated_at       │        │
│         │                                      └──────────────────┘        │
│         │               ┌──────────────────┐                               │
│         │               │ ba_verification  │                               │
│         │               ├──────────────────┤                               │
│         │               │ id (PK)          │                               │
│         │               │ identifier       │                               │
│         │               │ value            │                               │
│         │               │ expires_at       │                               │
│         │               │ created_at       │                               │
│         │               │ updated_at       │                               │
│         │               └──────────────────┘                               │
│         │                                                                   │
└─────────┼───────────────────────────────────────────────────────────────────┘
          │
          │ user_id reference (application level)
          ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          EXISTING TABLES (no prefix)                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────┐           ┌──────────────────┐                        │
│  │  chat_sessions   │           │  chat_messages   │                        │
│  ├──────────────────┤           ├──────────────────┤                        │
│  │ id (PK)          │◄──────────│ chat_session_id  │                        │
│  │ user_id          │           │ id (PK)          │                        │
│  │ title            │           │ sender           │                        │
│  │ created_at       │           │ content          │                        │
│  │ last_interaction │           │ sequence_number  │                        │
│  │ is_active        │           │ timestamp        │                        │
│  └──────────────────┘           └──────────────────┘                        │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Entity Definitions

### ba_user (Better Auth - Managed)

Represents a registered user in the system.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | TEXT | PRIMARY KEY | Unique identifier (UUID) |
| email | TEXT | UNIQUE, NOT NULL | User's email address |
| name | TEXT | NULLABLE | Display name |
| email_verified | BOOLEAN | DEFAULT FALSE | Whether email is verified |
| image | TEXT | NULLABLE | Profile image URL |
| created_at | TIMESTAMP | DEFAULT NOW() | Account creation time |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last update time |

**Validation Rules**:
- Email must be valid format (RFC 5322)
- Email must be unique across all users
- Name max length: 255 characters

---

### ba_session (Better Auth - Managed)

Represents an active user login session.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | TEXT | PRIMARY KEY | Session identifier |
| user_id | TEXT | FK → ba_user.id, NOT NULL | Owner of session |
| token | TEXT | UNIQUE, NOT NULL | Session token for auth |
| expires_at | TIMESTAMP | NOT NULL | Session expiration time |
| ip_address | TEXT | NULLABLE | Client IP address |
| user_agent | TEXT | NULLABLE | Client user agent string |
| created_at | TIMESTAMP | DEFAULT NOW() | Session creation time |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last activity time |

**Validation Rules**:
- Token must be unique
- expires_at must be in the future at creation
- Cascade delete when user is deleted

**State Transitions**:
- Active → Expired (time-based)
- Active → Revoked (user logout)
- Active → Refreshed (activity extends expiry)

---

### ba_account (Better Auth - Managed)

Links authentication methods to a user (password, OAuth providers).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | TEXT | PRIMARY KEY | Account link identifier |
| user_id | TEXT | FK → ba_user.id, NOT NULL | User this account belongs to |
| account_id | TEXT | NOT NULL | External account ID (email or OAuth ID) |
| provider_id | TEXT | NOT NULL | Provider name (credential, google) |
| access_token | TEXT | NULLABLE | OAuth access token |
| refresh_token | TEXT | NULLABLE | OAuth refresh token |
| access_token_expires_at | TIMESTAMP | NULLABLE | Token expiration |
| refresh_token_expires_at | TIMESTAMP | NULLABLE | Refresh token expiration |
| scope | TEXT | NULLABLE | OAuth scopes granted |
| password | TEXT | NULLABLE | Hashed password (credential provider only) |
| created_at | TIMESTAMP | DEFAULT NOW() | Link creation time |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last update time |

**Validation Rules**:
- One account per (user_id, provider_id) combination
- Password must be hashed (never stored plaintext)
- Password minimum length: 8 characters (validated pre-hash)

**Provider Types**:
- `credential`: Email/password authentication
- `google`: Google OAuth

---

### ba_verification (Better Auth - Managed)

Stores verification tokens (email verification, password reset - future).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | TEXT | PRIMARY KEY | Verification record ID |
| identifier | TEXT | NOT NULL | What is being verified (email) |
| value | TEXT | NOT NULL | Verification token value |
| expires_at | TIMESTAMP | NOT NULL | Token expiration |
| created_at | TIMESTAMP | DEFAULT NOW() | Token creation time |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last update time |

**Note**: Not used in initial implementation (email verification disabled).

---

### chat_sessions (Existing - Modified)

Represents a conversation thread between user and Fubuni.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | TEXT | PRIMARY KEY | Session UUID |
| user_id | TEXT | NOT NULL | **CHANGED**: Now stores ba_user.id instead of anonymous_* |
| title | TEXT | NULLABLE | Conversation title (from first message) |
| created_at | TIMESTAMP | DEFAULT NOW() | Session start time |
| last_interaction | TIMESTAMP | DEFAULT NOW() | Last message time |
| is_active | BOOLEAN | DEFAULT TRUE | Whether session is active |

**Migration Note**: Existing `anonymous_*` user_ids remain unchanged; new sessions will use authenticated user IDs.

---

### chat_messages (Existing - Unchanged)

Represents a single message in a conversation.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | TEXT | PRIMARY KEY | Message UUID |
| sender | TEXT | NOT NULL | 'user' or 'fubuni' |
| content | TEXT | NOT NULL | Message content |
| chat_session_id | TEXT | FK → chat_sessions.id | Parent session |
| sequence_number | INTEGER | NOT NULL | Order within session |
| timestamp | TIMESTAMP | DEFAULT NOW() | Message time |

---

## Indexes

### Better Auth Tables (Created by migration)

```sql
CREATE INDEX idx_ba_session_token ON ba_session(token);
CREATE INDEX idx_ba_session_user_id ON ba_session(user_id);
CREATE INDEX idx_ba_session_expires_at ON ba_session(expires_at);
CREATE INDEX idx_ba_account_user_id ON ba_account(user_id);
CREATE INDEX idx_ba_account_provider ON ba_account(provider_id, account_id);
CREATE INDEX idx_ba_user_email ON ba_user(email);
```

### Existing Tables (No changes needed)

Existing indexes on `chat_sessions` and `chat_messages` remain unchanged.

---

## Data Flow

### User Registration Flow

```
1. User submits email + password
2. ba_user record created
3. ba_account record created (provider_id='credential', password=hashed)
4. ba_session record created
5. Session token returned to client
```

### Google OAuth Flow

```
1. User clicks "Sign in with Google"
2. Redirect to Google OAuth
3. Google returns authorization code
4. Better Auth exchanges code for tokens
5. ba_user created/found (by email)
6. ba_account created/updated (provider_id='google')
7. ba_session created
8. Session token returned to client
```

### Chat Message Flow (Post-Auth)

```
1. Authenticated user sends message
2. FastAPI validates session token with auth-service
3. User ID extracted from session
4. chat_sessions record created/updated (user_id = ba_user.id)
5. chat_messages record created
6. Response generated and stored
```

---

## Migration Strategy

### New Tables

Better Auth CLI creates tables automatically:
```bash
cd auth-service && npx @better-auth/cli migrate
```

### Existing Data

- **No migration needed** for existing `chat_sessions`
- Existing `anonymous_*` user_ids remain valid
- New sessions will have authenticated user_ids
- Query patterns support both (WHERE user_id = ?)
