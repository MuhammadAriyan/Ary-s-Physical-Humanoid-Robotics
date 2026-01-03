# Implementation Plan: Consolidate Better Auth service into FastAPI backend

**Branch**: `011-auth-consolidation` | **Date**: 2026-01-03 | **Spec**: [link to spec.md]
**Input**: Feature specification from `/specs/011-auth-consolidation/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Replace Node.js Better Auth service with Python FastAPI auth endpoints while maintaining API compatibility with existing frontend. The implementation will use python-jose for JWT handling, passlib for password hashing, and maintain database schema compatibility with existing Better Auth schema. The unified service will handle both authentication and existing chat functionality with shared database for user and chat data.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: Python 3.11, TypeScript 5.6, React 18
**Primary Dependencies**: FastAPI, python-jose[cryptography], passlib[bcrypt], python-multipart, Docusaurus 3
**Storage**: PostgreSQL database (shared with chat functionality)
**Testing**: pytest for backend, Jest/React Testing Library for frontend
**Target Platform**: Linux server (containerized deployment)
**Project Type**: Web application (backend API with existing frontend integration)
**Performance Goals**: <2s authentication response time, support 100 concurrent auth requests
**Constraints**: Must maintain API compatibility with existing frontend, CORS support for GitHub Pages
**Scale/Scope**: Support existing user base and chat functionality integration

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Verify compliance with Fubuni Docs Agent Constitution:
- Agent MUST be named "Fubuni" and never hallucinate (respond with "I'm not sure yet, teach me!" when lacking knowledge) - **COMPLIANT**
- Implementation MUST use only OpenAI-compatible providers via OPENAI_BASE_URL (no direct API vendor locks) - **COMPLIANT**
- Backend MUST be in Python 3.11 using official OpenAI Agents SDK - **COMPLIANT**
- Frontend MUST perfectly match Docusaurus theme and use React + TypeScript - **COMPLIANT**
- Implementation MUST be fully static deployable with no Node server at runtime - **COMPLIANT** (replacing Node.js auth with Python backend)
- Initial implementation MUST NOT include vector DB, crawling, or embeddings - **COMPLIANT**
- System architecture MUST support later documentation ingestion - **COMPLIANT**

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── auth/
│   │   ├── models.py          # User, Session, Account models
│   │   ├── schemas.py         # Pydantic schemas for auth endpoints
│   │   ├── crud.py            # Database operations for auth
│   │   ├── security.py        # JWT, password hashing utilities
│   │   ├── router.py          # Auth API endpoints
│   │   └── dependencies.py    # Auth dependency functions
│   ├── chat/
│   │   ├── models.py          # Chat-related models
│   │   ├── router.py          # Chat API endpoints
│   │   └── services.py        # Chat business logic
│   ├── database/
│   │   ├── models.py          # Base models
│   │   ├── session.py         # Database session management
│   │   └── init.py            # Database initialization
│   ├── main.py                # FastAPI application entry point
│   └── config.py              # Configuration settings
└── tests/
    ├── auth/
    │   ├── test_auth.py       # Auth endpoint tests
    │   └── test_security.py   # Security tests
    ├── chat/
    │   └── test_chat.py       # Chat functionality tests
    └── conftest.py            # Test configuration

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

.env                            # Environment variables
Dockerfile                      # Container configuration
docker-compose.yml              # Multi-container setup
requirements.txt                # Python dependencies
```

**Structure Decision**: Web application with backend API and existing frontend integration. The auth service is integrated into the existing backend structure under the `auth/` module, sharing the same database and deployment as the chat functionality. This maintains the unified service approach while keeping clear separation of concerns between auth and chat modules.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
