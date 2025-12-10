# Implementation Plan: Fubuni Chat Bubble

**Branch**: `007-fubuni-chat-bubble` | **Date**: 2025-12-09 | **Spec**: [Fubuni Chat Bubble Feature Spec](specs/007-fubuni-chat-bubble/spec.md)
**Input**: Feature specification from `/specs/007-fubuni-chat-bubble/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a floating chat bubble UI component for Docusaurus documentation that provides instant access to the Fubuni AI assistant. The implementation includes a React-based frontend component that integrates with Docusaurus via swizzling, and a FastAPI backend using the OpenAI Agents SDK to process user queries. The solution must be fully static deployable and integrate seamlessly with the existing Docusaurus theme.

## Technical Context

**Language/Version**: Python 3.11, TypeScript/JavaScript, React 18
**Primary Dependencies**: FastAPI, OpenAI Agents SDK (agents package), Docusaurus 3, React, TypeScript
**Storage**: Neon Serverless Postgres database for storing chat sessions and messages
**Testing**: pytest for backend, Jest/React Testing Library for frontend
**Target Platform**: Web browser (client-side chat UI), Linux server (backend API)
**Project Type**: Web application (frontend + backend API)
**Performance Goals**: <300ms drawer open time, <5s first response time, token-by-token streaming
**Constraints**: Must work in static Docusaurus build, must inherit Docusaurus theme automatically
**Scale/Scope**: Single user sessions, concurrent users limited by backend capacity

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Verify compliance with Fubuni Docs Agent Constitution:
- Agent MUST be named "Fubuni" and never hallucinate (respond with "I'm not sure yet, teach me!" when lacking knowledge)
- Implementation MUST use only OpenAI-compatible providers via OPENAI_BASE_URL (no direct API vendor locks)
- Backend MUST be in Python 3.11 using official OpenAI Agents SDK
- Frontend MUST perfectly match Docusaurus theme and use React + TypeScript
- Implementation MUST be fully static deployable with no Node server at runtime
- Initial implementation MUST NOT include vector DB, crawling, or embeddings
- System architecture MUST support later documentation ingestion

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

Based on the user requirements, the implementation will follow a web application structure with separate backend and Docusaurus frontend:

```text
backend/
├── app/
│   ├── __init__.py
│   ├── main.py          # FastAPI application entry point
│   ├── api/
│   │   ├── __init__.py
│   │   ├── chat.py      # Chat endpoint with SSE streaming
│   │   └── models.py    # Pydantic models for request/response
│   ├── agents/
│   │   ├── __init__.py
│   │   ├── fubuni_agent.py  # Fubuni agent implementation
│   │   └── tools.py     # Agent tools
│   └── config/
│       ├── __init__.py
│       └── settings.py   # Configuration and settings
├── requirements.txt
└── tests/
    ├── __init__.py
    ├── test_chat.py
    └── conftest.py

src/
├── components/
│   └── FubuniChat/
│       ├── FubuniChat.tsx      # Main chat component
│       ├── FubuniBubble.tsx    # Floating bubble component
│       ├── ChatDrawer.tsx      # Slide-out drawer component
│       ├── ChatModal.tsx       # Full-screen modal component
│       ├── ChatMessage.tsx     # Message display component
│       ├── ChatInput.tsx       # Input component with streaming
│       └── styles.module.css   # Component-specific styles using Infima
└── theme/
    └── FubuniChatInjector/     # Docusaurus swizzled component
        └── index.js             # Injects Fubuni chat into Docusaurus

docusaurus.config.js            # Docusaurus configuration
package.json                   # Frontend dependencies
```

**Structure Decision**: Web application with separate backend API service and Docusaurus frontend integration. The backend uses FastAPI for the chat API endpoint with SSE streaming, while the frontend integrates with Docusaurus through swizzling to inject the chat bubble component. The structure allows for static deployment of the frontend while maintaining a separate backend service.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
