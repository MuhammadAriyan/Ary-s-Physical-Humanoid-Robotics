# Implementation Plan: Migrate Auth Service to Replit

**Branch**: `012-migrate-auth-replit` | **Date**: 2026-01-03 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/012-migrate-auth-replit/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Migrate the Better Auth + Express authentication service from Koyeb hosting to Replit hosting platform while maintaining all functionality and ensuring proper CORS configuration. This involves deploying the existing auth service code to Replit's Node.js environment, configuring CORS for frontend compatibility, and setting up environment variables securely.

## Technical Context

**Language/Version**: Node.js, Express framework, Better Auth library
**Primary Dependencies**: Express.js, Better Auth, and related auth dependencies
**Storage**: External database (likely Neon PostgreSQL) - existing from current setup
**Testing**: Environment-specific testing for deployment and CORS
**Target Platform**: Replit Node.js runtime environment
**Project Type**: web (backend API service for authentication)
**Performance Goals**: Acceptable response times for auth operations within Replit free tier constraints
**Constraints**: Replit free tier limitations (sleep/wake cycles, resource constraints)
**Scale/Scope**: Personal/project use, not production-scale deployment

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-checked after Phase 1 design.*

Verify compliance with project constitution for auth service:
- ✅ Migration preserves all existing authentication functionality
- ✅ Implementation maintains security standards of Better Auth
- ✅ Backend continues using Express.js and Better Auth as authentication framework
- ✅ Migration ensures CORS configuration allows frontend applications to connect
- ✅ Implementation securely handles environment variables and secrets
- ✅ System architecture remains compatible with existing frontend applications
- ✅ Migration does NOT introduce breaking changes to auth API contracts

## Project Structure

### Documentation (this feature)

```text
specs/012-migrate-auth-replit/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
auth-service/
├── src/
│   ├── index.js         # Main Express/Better Auth entry point
│   └── config/          # Auth configuration
├── package.json         # Dependencies and scripts
└── .env.example         # Environment variable documentation
```

**Structure Decision**: The auth service will be deployed as a Node.js application to Replit with the existing source structure. The service will maintain its current architecture as an authentication API service.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Replit hosting instead of original Koyeb | Koyeb free trial ended, need free alternative | Koyeb requires paid subscription for continued use |
