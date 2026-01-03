# Research: Auth Service Consolidation

## Decision: Replace Node.js Better Auth with Python FastAPI Auth Implementation
**Rationale**: Consolidate authentication service into the existing Python FastAPI backend to eliminate microservice complexity, reduce deployment overhead, and maintain a unified codebase. This approach aligns with the project's Python backend architecture and simplifies data management with a shared database.

**Alternatives considered**:
- Keep Node.js auth service with API communication to Python backend
- Use a completely different auth solution (Auth0, Firebase, etc.)
- Hybrid approach with some auth features in Python, others in Node.js

## Decision: JWT Token Implementation Strategy
**Rationale**: Use python-jose for JWT handling to maintain compatibility with existing frontend expectations while providing secure token generation and validation. This library provides similar functionality to Better Auth's token handling but in Python.

**Alternatives considered**:
- PyJWT (simpler but less comprehensive)
- Custom token implementation
- Session-based authentication only

## Decision: Database Schema Compatibility
**Rationale**: Maintain database schema compatibility with Better Auth to ensure smooth migration without data loss. This includes User, Session, and Account tables with appropriate fields and relationships.

**Alternatives considered**:
- Complete schema redesign
- Separate auth and chat databases
- NoSQL database for authentication data

## Decision: Password Hashing Algorithm
**Rationale**: Use passlib[bcrypt] for password hashing as it provides industry-standard security and is compatible with common best practices. Bcrypt is also used by Better Auth, ensuring compatibility.

**Alternatives considered**:
- Argon2 (more modern but requires additional dependencies)
- Scrypt (another secure option)
- SHA-256 with salt (less secure than bcrypt)

## Decision: CORS Configuration
**Rationale**: Configure CORS to support GitHub Pages integration while maintaining security. This includes proper origin validation and appropriate credential handling for authentication cookies.

**Alternatives considered**:
- Separate auth domain
- JWT-only approach (no cookies)
- More permissive CORS settings (less secure)

## Decision: API Endpoint Design
**Rationale**: Design API endpoints to maintain compatibility with existing frontend integration while following RESTful patterns. Endpoints will match Better Auth's expected interface to ensure frontend compatibility.

**Alternatives considered**:
- GraphQL instead of REST
- Different endpoint structure
- Versioned API endpoints