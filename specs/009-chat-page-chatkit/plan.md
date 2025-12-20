# Implementation Plan: Chat Page with Documentation Integration

**Branch**: `009-chat-page-chatkit` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/009-chat-page-chatkit/spec.md`

## Summary

Create a /chat page with split-screen interface: documentation viewer (iframe, left 60%) and OpenAI ChatKit chat interface (right 40%). Backend agent returns structured responses using Pydantic BaseModel with `output_type` parameter, including chapter references that trigger automatic documentation navigation. Mobile responsive with single-panel toggle.

## Technical Context

**Language/Version**: Python 3.11 (backend), TypeScript 5.6 (frontend)
**Primary Dependencies**:
- Backend: FastAPI, OpenAI Agents SDK, Pydantic v2
- Frontend: Docusaurus 3, React 19, @openai/chat-kit-react
**Storage**: N/A (session-based state, existing chat session DB)
**Testing**: pytest (backend), Playwright (frontend E2E)
**Target Platform**: Web (Docusaurus static site + HuggingFace Spaces backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <3s chat response, <500ms doc navigation, <2s page load
**Constraints**: Must be static deployable, iframe same-origin for docs
**Scale/Scope**: Single page (/chat), 5 doc chapters, existing chat infrastructure

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status | Notes |
|-----------|-------------|--------|-------|
| Agent Identity | Named "Fubuni", no hallucination | ✅ PASS | Existing agent already compliant |
| Provider Constraint | OpenAI-compatible via OPENAI_BASE_URL | ✅ PASS | Using OpenRouter, no vendor lock |
| Tech Stack Compliance | Python 3.11 + OpenAI Agents SDK backend | ✅ PASS | Existing infrastructure |
| Tech Stack Compliance | Docusaurus + React + TypeScript frontend | ✅ PASS | Native Docusaurus page |
| Static Deployability | No Node server at runtime | ✅ PASS | Docusaurus static build |
| Non-Goals Respect | No vector DB, crawling, embeddings in initial | ⚠️ N/A | RAG already exists, not adding new |
| Future Extensibility | Support later doc ingestion | ✅ PASS | Structured output enables future features |

**Gate Status**: ✅ PASS - All applicable principles satisfied

## Project Structure

### Documentation (this feature)

```text
specs/009-chat-page-chatkit/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── chat-api.yaml    # Updated chat endpoint contract
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── agents/
│   │   └── fubuni_agent.py    # MODIFY: Add AgentResponse model + output_type
│   ├── api/
│   │   ├── chat.py            # MODIFY: Update ChatResponse with chapter fields
│   │   └── models.py          # MODIFY: Add structured response models
│   └── config/
└── tests/

src/                           # Docusaurus frontend
├── pages/
│   ├── chat.tsx               # CREATE: New chat page
│   └── chat.module.css        # CREATE: Page styling
├── components/
│   └── FubuniChat/            # EXISTING: May reuse components
└── theme/
```

**Structure Decision**: Web application pattern - extends existing backend/frontend structure. New page at `src/pages/chat.tsx`, backend modifications to existing agent and API files.

## Complexity Tracking

No constitution violations - no justification needed.

## Phase 0: Research Summary

See [research.md](./research.md) for full details.

### Key Decisions

1. **Chat UI Library**: OpenAI ChatKit (@openai/chat-kit-react) for consistent chat UX
2. **Doc Embedding**: iframe with same-origin docs (Docusaurus baseUrl)
3. **Structured Output**: Pydantic BaseModel with OpenAI Agents SDK `output_type`
4. **State Management**: React useState for doc navigation state
5. **Mobile Strategy**: CSS media query with toggle button

## Phase 1: Design Artifacts

- [data-model.md](./data-model.md) - Entity definitions
- [contracts/chat-api.yaml](./contracts/chat-api.yaml) - Updated API contract
- [quickstart.md](./quickstart.md) - Implementation guide
