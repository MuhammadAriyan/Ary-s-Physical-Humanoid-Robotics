# Implementation Plan: Fubuni Web Search Integration

**Branch**: `010-fubuni-web-search` | **Date**: 2025-12-21 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/010-fubuni-web-search/spec.md` + User clarifications

## Summary

Add DuckDuckGo web search capability to Fubuni chat agent as a **lowest priority fallback** tool (after RAG knowledge base). When web search is used, results display in a **slide-in panel from the right** with the same close button UX as the docs panel. A special "use rag" flag in user input disables web search entirely.

## User Clarifications (from `/sp.plan` input)

1. **Priority**: Web search is **least priority** - only used when documentation has no results
2. **UI Behavior**: Web search sources panel slides from **right** (mirroring docs which slide from left)
3. **Close Button**: Same X button pattern as docs panel for dismissing web sources
4. **Triple-Panel Layout**: When docs + chat + web sources all visible: **40% docs | 30% chat | 30% sources**
5. **RAG Override**: When user input contains "use rag", web search tool is **disabled** for that request

## Technical Context

**Language/Version**: Python 3.12 (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, duckduckgo-search, React, Docusaurus 3
**Storage**: Neon PostgreSQL (existing chat sessions)
**Testing**: pytest (backend), manual testing (frontend)
**Target Platform**: Linux server (backend on HuggingFace Spaces), Static web (Vercel)
**Project Type**: Web application (backend + frontend)
**Performance Goals**: Web search response < 10 seconds
**Constraints**: No API keys required, graceful fallback on failure
**Scale/Scope**: Single user concurrent requests

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Verify compliance with Fubuni Docs Agent Constitution:
- [x] Agent MUST be named "Fubuni" and never hallucinate → ✅ Fubuni agent maintains identity, web search provides attributed sources
- [x] Implementation MUST use only OpenAI-compatible providers via OPENAI_BASE_URL → ✅ No change to LLM provider
- [x] Backend MUST be in Python 3.11+ using official OpenAI Agents SDK → ✅ Adding `duckduckgo-search` as new dependency
- [x] Frontend MUST perfectly match Docusaurus theme and use React + TypeScript → ✅ Web sources panel uses same design language as docs panel
- [x] Implementation MUST be fully static deployable with no Node server at runtime → ✅ No runtime Node.js changes
- [x] Initial implementation MUST NOT include vector DB, crawling, or embeddings → ⚠️ Web search is external API call, not local RAG (acceptable)
- [x] System architecture MUST support later documentation ingestion → ✅ Tool priority maintains RAG as primary

**Gate Status: PASS** - All constitution requirements satisfied.

## Project Structure

### Documentation (this feature)

```text
specs/010-fubuni-web-search/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── agents/
│   │   └── fubuni_agent.py    # ADD: search_web tool, RAG override detection
│   └── api/
│       ├── chat.py            # ADD: web_sources field in response
│       └── models.py          # ADD: WebSearchResult, update ChatResponse
├── requirements.txt           # ADD: duckduckgo-search
└── rag_retriever.py           # No changes

src/
├── pages/
│   ├── chat.tsx               # ADD: WebSourcesPanel component, triple layout
│   └── chat.module.css        # ADD: .webSourcesPanel, .withAll styles
└── constants/
    └── chapters.ts            # No changes
```

**Structure Decision**: Web application structure retained. Backend adds new tool to existing agent. Frontend adds new panel component mirroring existing docs panel pattern.

## Complexity Tracking

No constitution violations requiring justification.

## Architecture Overview

### Tool Priority Chain

```
User Question
    │
    ├── Check for "use rag" flag → If found, SKIP web search entirely
    │
    ▼
┌─────────────────────────┐
│ 1. search_knowledge_base│  (PRIMARY - RAG)
└───────────┬─────────────┘
            │ No results?
            ▼
┌─────────────────────────┐
│ 2. search_web           │  (SECONDARY - DuckDuckGo)
└───────────┬─────────────┘
            │ No results or error?
            ▼
┌─────────────────────────┐
│ 3. get_robotics_info    │  (FALLBACK - General knowledge)
└─────────────────────────┘
```

### Frontend Layout States

```
State 1: Chat Only (default)
┌────────────────────────────┐
│         Chat 100%          │
└────────────────────────────┘

State 2: With Docs (slide from left)
┌───────────────┬────────────┐
│   Docs 60%    │  Chat 40%  │
└───────────────┴────────────┘

State 3: With Web Sources (slide from right)
┌────────────────┬───────────┐
│    Chat 70%    │ Sources   │
│                │   30%     │
└────────────────┴───────────┘

State 4: All Three Panels
┌──────────┬─────────┬───────┐
│ Docs 40% │Chat 30% │Sources│
│          │         │ 30%   │
└──────────┴─────────┴───────┘
```

### Response Schema Extension

```typescript
interface ChatResponse {
  response: string;
  session_id: string;
  timestamp: string;
  // Existing navigation
  chapter?: string;
  section?: string;
  should_navigate?: boolean;
  // NEW: Web search results
  web_sources?: WebSearchResult[];
  used_web_search?: boolean;
}

interface WebSearchResult {
  title: string;
  url: string;
  snippet: string;
}
```

## Key Design Decisions

### D1: RAG Override Mechanism

**Decision**: Check user message for substring "use rag" (case-insensitive) before tool selection.

**Rationale**: Simple, user-controllable override without complex settings. User explicitly signals preference for documentation-only responses.

**Implementation**: In agent instructions, add conditional logic to skip `search_web` when override detected.

### D2: Web Sources Panel Position

**Decision**: Slide from right edge, opposite to docs panel.

**Rationale**:
- Visual symmetry with docs panel
- Clear spatial separation: docs=left, chat=center, sources=right
- Maintains existing docs panel behavior unchanged

### D3: Triple Layout Proportions (40-30-30)

**Decision**: Docs 40%, Chat 30%, Sources 30% when all panels visible.

**Rationale**:
- Docs get most space as they contain comprehensive content
- Chat and sources split remaining 60% equally (30% each)
- Chat still usable at 30% for follow-up questions
- Rare state - user typically views one or the other

### D4: DuckDuckGo Library

**Decision**: Use `duckduckgo-search` Python library (no API key required).

**Rationale**:
- Free, no API key needed
- Maintained library with good search quality
- Returns structured results (title, url, body)
- 5 results limit per search (configurable)
