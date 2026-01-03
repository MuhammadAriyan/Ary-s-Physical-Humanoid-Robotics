# ADR-0001: Auth Service Consolidation Approach

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2026-01-03
- **Feature:** 011-auth-consolidation
- **Context:** Replace Node.js Better Auth service with Python FastAPI auth endpoints while maintaining API compatibility with existing frontend. This decision impacts the overall architecture by consolidating authentication into the existing Python backend, affecting deployment, security, and data management.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- **Backend Framework**: Python 3.11 with FastAPI for authentication endpoints
- **JWT Implementation**: python-jose[cryptography] for token handling
- **Password Hashing**: passlib[bcrypt] for secure password storage
- **Database**: PostgreSQL with schema compatibility to Better Auth
- **Security**: HttpOnly, Secure, SameSite=Lax cookies for authentication
- **Token Expiration**: 15 minutes for access tokens, 1 day for refresh tokens
- **Social Login**: Google and GitHub OAuth providers only
- **Deployment**: Single container deployment combining auth and chat functionality

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Reduced operational complexity by eliminating microservice architecture
- Unified codebase in Python, simplifying maintenance and development
- Shared database eliminates data synchronization issues
- Consistent security model across auth and chat functionality
- Lower deployment overhead with single container approach
- Maintained API compatibility with existing frontend

### Negative

- Increased coupling between authentication and chat services
- Single point of failure for both auth and chat functionality
- Potential performance impact if auth service experiences high load
- Migration complexity from existing Better Auth schema
- Increased complexity in the single backend service

## Alternatives Considered

**Alternative A**: Keep Node.js Better Auth service with API communication to Python backend
- Why rejected: Would maintain microservice complexity and introduce cross-service communication overhead

**Alternative B**: Use external auth service (Auth0, Firebase, etc.)
- Why rejected: Would introduce vendor lock-in and external dependency for core functionality

**Alternative C**: Hybrid approach with some auth features in Python, others in Node.js
- Why rejected: Would create inconsistent architecture and increase maintenance complexity

## References

- Feature Spec: /specs/011-auth-consolidation/spec.md
- Implementation Plan: /specs/011-auth-consolidation/plan.md
- Related ADRs: None
- Evaluator Evidence: /history/prompts/011-auth-consolidation/ (PHR records)
