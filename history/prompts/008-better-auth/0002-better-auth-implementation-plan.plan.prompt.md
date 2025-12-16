---
id: "0002"
title: "Better Auth Implementation Plan"
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 008-better-auth
branch: 008-better-auth
user: ary
command: /sp.plan
labels: ["authentication", "better-auth", "microservice", "planning", "architecture"]
links:
  spec: specs/008-better-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/008-better-auth/plan.md
  - specs/008-better-auth/research.md
  - specs/008-better-auth/data-model.md
  - specs/008-better-auth/quickstart.md
  - specs/008-better-auth/contracts/auth-service-api.yaml
  - specs/008-better-auth/contracts/fastapi-chat-api.yaml
tests:
  - null
---

## Prompt

Create an implementation plan for the Better Auth integration feature based on the specification.

Technical context from codebase exploration:

FRONTEND:
- Entry point: src/theme/Root.tsx (wraps entire app)
- Chat component: src/components/FubuniChat/FubuniChat.tsx (168 lines)
- Backend URL detection: src/theme/FubuniChatInjector/index.js
- API calls use native fetch() with no abstraction layer
- Session currently stored in React useState (lost on refresh)
- No existing auth libraries in package.json
- Uses localStorage for theme preference (pattern to follow)

BACKEND:
- Main app: backend/app/main.py (CORS middleware only, no auth)
- Settings: backend/app/config/settings.py (Pydantic BaseSettings pattern)
- Chat API: backend/app/api/chat.py (endpoints: POST /api/chat, GET /api/chat/sessions, GET /api/chat/history/{id})
- Models: backend/app/models/__init__.py (ChatSession with user_id field currently set to "anonymous_*")
- Database: Neon PostgreSQL via SQLModel, connection in backend/app/config/database.py
- Dependency injection pattern: Depends(get_session) for DB access

CONSTRAINTS:
- Must not break existing functionality
- Auth service must be isolated in auth-service/ folder
- Use ba_ prefix for Better Auth database tables to avoid conflicts
- Keep /health and /api/rag/* endpoints public (no auth required)
- Frontend auth client must handle SSR (Docusaurus)
- Support both localhost development and production URLs

IMPLEMENTATION PHASES:
1. Auth service setup (new Node.js service with Better Auth + Express)
2. Database migrations (create ba_user, ba_session, ba_account, ba_verification tables)
3. Frontend auth client and AuthProvider context
4. Auth UI components (modal with login/signup forms, Google button)
5. FastAPI JWT validation middleware
6. Protect chat endpoints with require_auth dependency
7. Update FubuniChat to require auth and send JWT headers

Deployment target: Hugging Face Spaces for auth service

## Response snapshot

Created comprehensive implementation plan with:
- 7 implementation phases with acceptance criteria
- Technical research document (research.md) with 9 decisions
- Data model (data-model.md) with 6 entity definitions
- API contracts (OpenAPI 3.0) for auth-service and FastAPI
- Quickstart guide with step-by-step setup instructions
- Constitution compliance check with justified violations
- Risk analysis with 4 identified risks and mitigations

## Outcome

- Impact: Complete architectural plan ready for task generation
- Tests: N/A (planning phase)
- Files: 6 files created (plan.md, research.md, data-model.md, quickstart.md, 2 contract files)
- Next prompts: /sp.tasks to generate implementation tasks
- Reflection: Plan addresses all spec requirements with clear phases and acceptance criteria

## Evaluation notes (flywheel)

- Failure modes observed: Constitution violations identified and justified
- Graders run and results (PASS/FAIL): Constitution check PASS with justified violations
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
