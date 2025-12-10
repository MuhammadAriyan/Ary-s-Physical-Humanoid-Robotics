<!--
Sync Impact Report:
- Version change: N/A → 1.0.0
- Added principles: Agent Identity, Provider Constraint, Tech Stack Compliance, Static Deployability, Non-Goals Respect, Future Extensibility
- Added sections: Technology Stack Requirements, Development Workflow
- Templates requiring updates: ✅ .specify/templates/plan-template.md updated
- Follow-up TODOs: None
-->
# Fubuni Docs Agent Constitution
<!-- Example: Spec Constitution, TaskFlow Constitution, etc. -->

## Core Principles

### Agent Identity
<!-- Example: I. Library-First -->
Fubuni agent MUST always refer to itself by the name "Fubuni"; Fubuni agent MUST NOT hallucinate information and MUST respond with "I'm not sure yet, teach me!" when it lacks knowledge about a topic
<!-- Example: Every feature starts as a standalone library; Libraries must be self-contained, independently testable, documented; Clear purpose required - no organizational-only libraries -->

### Provider Constraint
<!-- Example: II. CLI Interface -->
Fubuni agent MUST use only OpenAI-compatible providers (Groq, Fireworks, OpenRouter, Together, local LLM) configured via OPENAI_BASE_URL environment variable; Direct integration with proprietary APIs is PROHIBITED
<!-- Example: Every library exposes functionality via CLI; Text in/out protocol: stdin/args → stdout, errors → stderr; Support JSON + human-readable formats -->

### Tech Stack Compliance
<!-- Example: III. Test-First (NON-NEGOTIABLE) -->
Backend MUST be implemented in Python 3.11 using the official OpenAI Agents SDK; Frontend MUST perfectly match Docusaurus theme (colors, fonts, dark/light mode, rounded corners, Infima classes) and use React + TypeScript
<!-- Example: TDD mandatory: Tests written → User approved → Tests fail → Then implement; Red-Green-Refactor cycle strictly enforced -->

### Static Deployability
<!-- Example: IV. Integration Testing -->
Fubuni implementation MUST be fully static deployable with no Node server required at runtime; Initial implementation MUST NOT include retrieval/augmented generation (RAG) capabilities (will be added in later phases)
<!-- Example: Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas -->

### Non-Goals Respect
<!-- Example: V. Observability, VI. Versioning & Breaking Changes, VII. Simplicity -->
Initial implementation MUST NOT include vector databases, web crawling, or embedding generation; Implementation MUST focus only on agent skeleton and perfect UI implementation
<!-- Example: Text I/O ensures debuggability; Structured logging required; Or: MAJOR.MINOR.BUILD format; Or: Start simple, YAGNI principles -->

### Future Extensibility
System architecture MUST be designed to support later ingestion of documentation and code examples; Implementation MUST provide extension points for future RAG capabilities

## Technology Stack Requirements
<!-- Example: Additional Constraints, Security Requirements, Performance Standards, etc. -->

Backend: Python 3.11 + FastAPI + official OpenAI Agents SDK; Frontend: Docusaurus 3 + React + TypeScript with swizzled components; Provider: Custom OpenAI-compatible providers only (no direct API vendor locks)
<!-- Example: Technology stack requirements, compliance standards, deployment policies, etc. -->

## Development Workflow
<!-- Example: Development Workflow, Review Process, Quality Gates, etc. -->

Fubuni UI MUST implement as a floating chat bubble in bottom-right corner that slides out a drawer interface; UI MUST support maximize to full-screen and minimize back to bubble states; UI implementation MUST use Docusaurus swizzled components
<!-- Example: Code review requirements, testing gates, deployment approval process, etc. -->

## Governance
<!-- Example: Constitution supersedes all other practices; Amendments require documentation, approval, migration plan -->

This Constitution supersedes all other development practices and requirements; Amendments require explicit documentation, team approval, and migration plan; All implementations MUST verify compliance with Fubuni-specific requirements during code reviews; Versioning follows semantic versioning (MAJOR.MINOR.PATCH) where MAJOR indicates breaking governance changes, MINOR indicates new principles, PATCH indicates clarifications
<!-- Example: All PRs/reviews must verify compliance; Complexity must be justified; Use [GUIDANCE_FILE] for runtime development guidance -->

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
<!-- Example: Version: 2.1.1 | Ratified: 2025-06-13 | Last Amended: 2025-07-16 -->