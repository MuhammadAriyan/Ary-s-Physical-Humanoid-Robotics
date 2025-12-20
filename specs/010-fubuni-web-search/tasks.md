# Tasks: Fubuni Web Search Integration

**Input**: Design documents from `/specs/010-fubuni-web-search/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are NOT included as they were not explicitly requested in the specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (Python/FastAPI)
- **Frontend**: `src/pages/` (React/TypeScript/Docusaurus)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Add duckduckgo-search dependency and project preparation

- [x] T001 Add `duckduckgo-search>=6.0.0` to backend/requirements.txt
- [x] T002 Install dependency with `pip install duckduckgo-search` in backend venv
- [x] T003 Verify installation with test import `from duckduckgo_search import DDGS`

---

## Phase 2: Foundational (Backend Models & API Schema)

**Purpose**: Create data models that ALL user stories depend on

**CRITICAL**: These changes must be complete before implementing any user story

- [x] T004 Add WebSearchResult Pydantic model in backend/app/api/models.py
- [x] T005 Extend ChatResponse model with web_sources and used_web_search fields in backend/app/api/models.py
- [x] T006 Add WebSearchResult TypeScript interface in src/pages/chat.tsx
- [x] T007 [P] Extend BackendResponse interface with web_sources and used_web_search in src/pages/chat.tsx

**Checkpoint**: Foundation ready - data models available for all user stories

---

## Phase 3: User Story 1 - Web Search Fallback for Missing Documentation (Priority: P1) MVP

**Goal**: Enable Fubuni to search the web when documentation lacks relevant results

**Independent Test**: Ask Fubuni "What is Tesla Optimus Gen 3?" and verify web search results with source URLs are returned

### Implementation for User Story 1

- [x] T008 [US1] Import DDGS from duckduckgo_search in backend/app/agents/fubuni_agent.py
- [x] T009 [US1] Create search_web function tool with @function_tool decorator in backend/app/agents/fubuni_agent.py
- [x] T010 [US1] Implement error handling with try/catch and graceful fallback message in search_web tool
- [x] T011 [US1] Format web search results with title, snippet, and source URL in search_web tool
- [x] T012 [US1] Extend AgentResponse model with web_sources and used_web_search fields in backend/app/agents/fubuni_agent.py
- [x] T013 [US1] Add search_web to agent tools list (position 3: after RAG, before general knowledge) in backend/app/agents/fubuni_agent.py
- [x] T014 [US1] Update agent instructions for web search usage with source prefix "Source: Web Search" in backend/app/agents/fubuni_agent.py
- [x] T015 [US1] Pass web_sources from agent response to ChatResponse in backend/app/api/chat.py

**Checkpoint**: Backend web search functional - test with API call to verify web results returned

---

## Phase 4: User Story 2 - Documentation Priority Over Web Search (Priority: P1)

**Goal**: Ensure RAG/documentation is always searched first and preferred over web search

**Independent Test**: Ask "What are humanoid robot sensors?" and verify documentation is used, not web search

### Implementation for User Story 2

- [x] T016 [US2] Update agent instructions to explicitly state "ALWAYS use search_knowledge_base FIRST" in backend/app/agents/fubuni_agent.py
- [x] T017 [US2] Add instruction "ONLY use search_web if search_knowledge_base returns no results" in backend/app/agents/fubuni_agent.py
- [x] T018 [US2] Add "use rag" override detection - skip web search if message contains "use rag" in agent instructions
- [x] T019 [US2] Update tool priority order in instructions: 1) search_knowledge_base 2) search_knowledge_base_detailed 3) search_web 4) get_robotics_info

**Checkpoint**: Tool priority enforced - documentation results preferred over web search

---

## Phase 5: User Story 3 - Graceful Handling of Web Search Failures (Priority: P2)

**Goal**: Fallback gracefully when web search fails or returns empty results

**Independent Test**: Simulate web search failure (disconnect network) and verify fallback to general knowledge without error message

### Implementation for User Story 3

- [x] T020 [US3] Add logging.warning for web search failures in search_web tool in backend/app/agents/fubuni_agent.py
- [x] T021 [US3] Return user-friendly message "Web search unavailable" instead of exception details
- [x] T022 [US3] Add instruction for agent to use get_robotics_info when web search returns no results
- [x] T023 [US3] Handle empty web_sources gracefully in frontend (don't show panel if empty) in src/pages/chat.tsx

**Checkpoint**: Web search failures handled gracefully with fallback to general knowledge

---

## Phase 6: Frontend Web Sources Panel (All User Stories)

**Goal**: Add slide-in panel from right to display web search sources

**Independent Test**: Send message triggering web search, verify panel slides from right with close button

### Implementation for Frontend Panel

- [x] T024 [P] Add sourcesVisible state variable in src/pages/chat.tsx
- [x] T025 [P] Add webSources state variable to store WebSearchResult[] in src/pages/chat.tsx
- [x] T026 Create WebSourcesPanel component with source list and close button in src/pages/chat.tsx
- [x] T027 Handle backend response - set webSources and sourcesVisible when used_web_search is true in src/pages/chat.tsx
- [x] T028 Add handleCloseSources callback to hide panel in src/pages/chat.tsx
- [x] T029 Update container className logic for layout states (chatOnly, withDocs, withSources, withAll) in src/pages/chat.tsx

**Checkpoint**: Web sources panel component ready - needs CSS styling

---

## Phase 7: Frontend CSS Styling

**Goal**: Style web sources panel with slide-from-right animation and triple layout

### Implementation for CSS

- [x] T030 [P] Add .webSourcesPanel base styles in src/pages/chat.module.css
- [x] T031 [P] Add .closeSourcesButton styles (mirror .closeDocsButton) in src/pages/chat.module.css
- [x] T032 [P] Add .withSources layout styles (grid-template-columns: 70% 30%) in src/pages/chat.module.css
- [x] T033 [P] Add .withAll layout styles (grid-template-columns: 40% 30% 30%) in src/pages/chat.module.css
- [x] T034 Add slide animation from right (transform: translateX(100%)) in src/pages/chat.module.css
- [x] T035 Add source list item styles (.sourceItem, .sourceTitle, .sourceSnippet, .sourceUrl) in src/pages/chat.module.css
- [x] T036 Add dark mode variants for all new styles in src/pages/chat.module.css
- [x] T037 Add responsive styles for mobile (stack vertically when all panels visible) in src/pages/chat.module.css

**Checkpoint**: Web sources panel fully styled with animations

---

## Phase 8: Polish & Integration

**Purpose**: Final integration and validation

- [x] T038 Test full flow: RAG miss → web search → sources panel display
- [x] T039 Test "use rag" override prevents web search
- [x] T040 Test triple-panel layout proportions (40-30-30)
- [x] T041 Test panel close button dismisses sources panel
- [x] T042 Run quickstart.md validation scenarios
- [ ] T043 Deploy backend to HuggingFace Spaces and test production

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - Core web search feature
- **User Story 2 (Phase 4)**: Depends on Phase 3 - Refines tool priority
- **User Story 3 (Phase 5)**: Depends on Phase 3 - Adds error handling
- **Frontend Panel (Phase 6)**: Depends on Foundational - Can run in parallel with backend stories
- **CSS Styling (Phase 7)**: Depends on Phase 6 - Panel must exist first
- **Polish (Phase 8)**: Depends on all phases complete

### User Story Dependencies

- **US1**: Independent after Foundational - Core feature
- **US2**: Builds on US1 (refines instructions) - Can be done with US1
- **US3**: Builds on US1 (adds error handling) - Can be done with US1
- **Frontend**: Independent of backend stories after Foundational

### Parallel Opportunities

**Phase 2 (Foundational)**:
```
T004 + T005 (both in models.py - sequential)
T006 + T007 (both in chat.tsx - sequential)
Backend models || Frontend interfaces (parallel)
```

**Phase 6 (Frontend Panel)**:
```
T024 + T025 (state variables - parallel)
```

**Phase 7 (CSS)**:
```
T030 + T031 + T032 + T033 (different CSS blocks - parallel)
```

---

## Implementation Strategy

### MVP First (Phases 1-5)

1. Complete Phase 1: Setup (install dependency)
2. Complete Phase 2: Foundational (data models)
3. Complete Phase 3: User Story 1 (web search works)
4. **STOP and VALIDATE**: Test API returns web results
5. Complete Phase 4 + 5: Refine priority and error handling

### Full Feature (Add Frontend)

6. Complete Phase 6: Frontend panel component
7. Complete Phase 7: CSS styling with animations
8. Complete Phase 8: Integration testing

### Deployment

1. Backend first → HuggingFace Spaces
2. Frontend second → Vercel
3. End-to-end testing in production

---

## Summary

| Metric | Count |
|--------|-------|
| Total Tasks | 43 |
| Phase 1 (Setup) | 3 |
| Phase 2 (Foundational) | 4 |
| Phase 3 (US1 - Web Search) | 8 |
| Phase 4 (US2 - Priority) | 4 |
| Phase 5 (US3 - Error Handling) | 4 |
| Phase 6 (Frontend Panel) | 6 |
| Phase 7 (CSS) | 8 |
| Phase 8 (Polish) | 6 |
| Parallel Tasks | 12 |

**MVP Scope**: Phases 1-5 (Backend only) = 23 tasks
**Full Feature**: All phases = 43 tasks
