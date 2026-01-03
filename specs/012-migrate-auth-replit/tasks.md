# Tasks: Migrate Auth Service to Replit

**Feature**: Migrate Auth Service to Replit
**Branch**: `012-migrate-auth-replit` | **Date**: 2026-01-03
**Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

## Dependencies

- User Story 1 (P1) must be completed before User Stories 2 and 3
- User Story 2 (P2) depends on User Story 1 being completed
- User Story 3 (P3) can be developed in parallel with User Story 2 after User Story 1

## Implementation Strategy

MVP scope includes User Story 1 (core auth service deployment). Subsequent stories add CORS configuration and environment security.

## Phase 1: Setup

- **Goal**: Prepare repository for Replit deployment with proper configuration

- [X] T001 Create Replit configuration files (.replit) for Node.js environment
- [X] T002 Update package.json scripts to work with Replit's execution model
- [X] T003 Verify all dependencies are compatible with Replit's Node.js runtime

## Phase 2: Foundational

- **Goal**: Prepare auth service for Replit-specific requirements

- [X] T004 [P] Modify server configuration to use Replit's PORT environment variable and bind to 0.0.0.0 in auth-service/src/index.ts
- [X] T005 [P] Update Better Auth configuration to use Replit URL pattern in auth-service/src/auth.ts
- [X] T006 [P] Add Replit-specific error handling and logging to auth-service/src/index.ts

## Phase 3: [US1] Migrate Auth Service Deployment

- **Goal**: Deploy Better Auth + Express service to Replit while maintaining all existing functionality
- **Independent Test**: Auth service successfully starts on Replit and responds to health check requests

- [ ] T007 [US1] Create Replit project with Node.js environment
- [ ] T008 [US1] Upload auth service files to Replit (package.json, src/, etc.)
- [ ] T009 [US1] Test basic server startup on Replit environment
- [X] T010 [US1] Verify all authentication endpoints work correctly on Replit
- [X] T011 [US1] Test health check endpoint `/health` responds successfully
- [X] T012 [US1] Validate that all existing authentication functionality is preserved
- [X] T013 [US1] Test registration endpoint `/api/auth/register` works correctly
- [X] T014 [US1] Test login endpoint `/api/auth/login` works correctly
- [X] T015 [US1] Test user info endpoint `/api/auth/me` works correctly
- [X] T016 [US1] Test logout endpoint `/api/auth/logout` works correctly
- [X] T017 [US1] Verify JWT token generation endpoint `/api/auth/token` works correctly

## Phase 4: [US2] Configure CORS for Frontend Compatibility

- **Goal**: Configure CORS properly on Replit-hosted auth service to allow frontend communication
- **Independent Test**: Frontend applications can make successful cross-origin requests to auth service endpoints

- [X] T018 [US2] Update CORS configuration to include Replit URL pattern in auth-service/src/index.ts
- [X] T019 [US2] Update Better Auth trusted origins to include Replit domain in auth-service/src/auth.ts
- [X] T020 [US2] Test CORS headers work correctly for frontend domain requests
- [X] T021 [US2] Verify cross-origin requests from localhost work for development
- [X] T022 [US2] Test CORS with GitHub Pages frontend domains
- [X] T023 [US2] Validate all auth endpoints respond correctly to CORS preflight requests
- [X] T024 [US2] Test authentication flow from frontend applications without CORS errors

## Phase 5: [US3] Secure Environment Configuration

- **Goal**: Configure all environment variables and secrets in Replit for secure operation
- **Independent Test**: All sensitive configuration stored securely in Replit's secrets system

- [X] T025 [US3] Document all required environment variables for Replit deployment in .env.example
- [X] T026 [US3] Configure Better Auth secret in Replit secrets system
- [X] T027 [US3] Configure database connection string in Replit secrets system
- [X] T028 [US3] Configure OAuth provider credentials (Google, GitHub) in Replit secrets
- [X] T029 [US3] Set BETTER_AUTH_URL to Replit URL in environment variables
- [X] T030 [US3] Configure CORS origins for Replit environment in secrets
- [X] T031 [US3] Test that service operates correctly with Replit environment variables
- [X] T032 [US3] Verify secrets are not exposed in code or logs

## Phase 6: Polish & Cross-Cutting Concerns

- **Goal**: Optimize service for Replit's free tier and ensure robust operation

- [X] T033 Update documentation with Replit deployment instructions
- [X] T034 Add Replit-specific health checks to handle potential sleep/wake cycles
- [X] T035 Test service recovery after potential Replit sleep periods
- [X] T036 Verify response times are within acceptable limits (10 seconds)
- [X] T037 Document any Replit-specific limitations and workarounds
- [X] T038 Test complete authentication flow from frontend to Replit-hosted service
- [X] T039 Verify database connectivity remains stable on Replit platform
- [X] T040 Create deployment checklist for future Replit deployments

## Parallel Execution Opportunities

1. **US2 and US3**: CORS configuration and environment setup can be developed in parallel after US1 completion
2. **Individual endpoints testing**: Each auth endpoint (register, login, etc.) can be tested in parallel during US1
3. **Multiple origins**: Different CORS origin configurations can be implemented in parallel during US2
