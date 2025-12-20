# Tasks: Chat Page with Documentation Integration

**Input**: Design documents from `/specs/009-chat-page-chatkit/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not explicitly requested - test tasks omitted

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (Python 3.11 + FastAPI)
- **Frontend**: `src/` (Docusaurus 3 + React + TypeScript)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Install dependencies and prepare project structure

- [x] T001 Install @openai/chat-kit-react dependency via npm install in package.json
- [x] T002 [P] Create chapter constants file at src/constants/chapters.ts with DOC_CHAPTERS mapping
- [x] T003 [P] Verify backend dependencies (Pydantic v2, OpenAI Agents SDK) in backend/requirements.txt

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Backend structured output - MUST complete before ANY frontend user story

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Add AgentResponse Pydantic model with chapter fields in backend/app/agents/fubuni_agent.py
- [x] T005 Add VALID_CHAPTERS constant list in backend/app/agents/fubuni_agent.py
- [x] T006 Update Agent initialization with output_type=AgentResponse in backend/app/agents/fubuni_agent.py
- [x] T007 Update agent instructions with chapter mapping keywords in backend/app/agents/fubuni_agent.py
- [x] T008 Update ChatResponse model with chapter, section, should_navigate fields in backend/app/api/models.py
- [x] T009 Update chat endpoint to return structured fields from AgentResponse in backend/app/api/chat.py
- [x] T010 Test backend structured output locally with verbose logging

**Checkpoint**: Backend returns structured responses with chapter references

---

## Phase 3: User Story 1 - Chat While Reading Documentation (Priority: P1) 🎯 MVP

**Goal**: Split-screen interface with auto-navigating documentation based on AI responses

**Independent Test**: Visit /chat, send "tell me about sensors", verify docs navigate to sensors chapter

### Implementation for User Story 1

- [x] T011 [P] [US1] Create chat page component at src/pages/chat.tsx with basic Layout wrapper
- [x] T012 [P] [US1] Create chat page styles at src/pages/chat.module.css with grid layout (60%/40% split)
- [x] T013 [US1] Add iframe documentation viewer component in src/pages/chat.tsx
- [x] T014 [US1] Add chat panel container with placeholder in src/pages/chat.tsx
- [x] T015 [US1] Implement ChatKit message list and input in src/pages/chat.tsx
- [x] T016 [US1] Add currentChapter state and handleAgentResponse function in src/pages/chat.tsx
- [x] T017 [US1] Connect chat to backend API (/api/chat) with fetch in src/pages/chat.tsx
- [x] T018 [US1] Implement auto-navigation: update iframe src when response.should_navigate is true
- [x] T019 [US1] Add smooth transition CSS for iframe src changes in src/pages/chat.module.css
- [x] T020 [US1] Wrap component in BrowserOnly for SSR compatibility in src/pages/chat.tsx

**Checkpoint**: User Story 1 complete - split-screen chat with auto-navigation works

---

## Phase 4: User Story 2 - Manual Documentation Navigation (Priority: P2)

**Goal**: Chapter tabs/selector for manual navigation independent of chat

**Independent Test**: Click chapter tabs, verify docs change without affecting chat history

### Implementation for User Story 2

- [x] T021 [P] [US2] Add chapter tabs container above iframe in src/pages/chat.tsx
- [x] T022 [P] [US2] Add chapter tab styles (.chapterTabs, .tab, .tab.active) in src/pages/chat.module.css
- [x] T023 [US2] Map DOC_CHAPTERS to clickable tab buttons in src/pages/chat.tsx
- [x] T024 [US2] Implement tab click handler to update currentChapter state in src/pages/chat.tsx
- [x] T025 [US2] Add active state styling to current chapter tab in src/pages/chat.tsx
- [x] T026 [US2] Ensure chat messages state persists when manually changing chapters in src/pages/chat.tsx

**Checkpoint**: User Story 2 complete - manual chapter navigation works alongside chat

---

## Phase 5: User Story 3 - Mobile Responsive Experience (Priority: P3)

**Goal**: Single-panel view with toggle button on mobile (< 768px)

**Independent Test**: Resize to mobile width, verify single panel with toggle between chat/docs

### Implementation for User Story 3

- [x] T027 [P] [US3] Add isMobileDocVisible state in src/pages/chat.tsx
- [x] T028 [P] [US3] Add mobile toggle button component in src/pages/chat.tsx
- [x] T029 [US3] Add mobile media query (@media max-width: 768px) in src/pages/chat.module.css
- [x] T030 [US3] Hide docViewer by default on mobile, show chatPanel in src/pages/chat.module.css
- [x] T031 [US3] Add .visible class for docViewer when toggle active in src/pages/chat.module.css
- [x] T032 [US3] Implement toggle handler to switch isMobileDocVisible state in src/pages/chat.tsx
- [x] T033 [US3] Style mobile toggle button (.mobileToggle) in src/pages/chat.module.css
- [x] T034 [US3] Test mobile layout at various viewport widths

**Checkpoint**: User Story 3 complete - mobile responsive with toggle works

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, edge cases, and deployment

- [x] T035 [P] Handle invalid chapter references from AI (fallback to current chapter) in src/pages/chat.tsx
- [x] T036 [P] Add loading state/skeleton for iframe while doc loads in src/pages/chat.tsx
- [x] T037 [P] Add error handling for failed chat API requests in src/pages/chat.tsx
- [x] T038 Add /chat link to navbar in docusaurus.config.ts
- [ ] T039 Deploy frontend to GitHub Pages via existing workflow
- [ ] T040 Deploy backend to Hugging Face Spaces via huggingface_hub
- [ ] T041 Verify full integration in production environment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all frontend user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Depends on Foundational (backend structured output)
- **User Story 2 (P2)**: Can start after US1 T013-T014 (needs iframe in place)
- **User Story 3 (P3)**: Can start after US1 complete (needs base layout)

### Within Each Phase

```
Phase 2 (Backend):
T004 → T005 → T006 → T007 (agent changes sequential)
T008 (parallel - different file)
T009 (depends on T006, T008)
T010 (depends on T009)

Phase 3 (US1):
T011, T012 → T013, T014 → T015, T016 → T017 → T018 → T019 → T020

Phase 4 (US2):
T021, T022 → T023 → T024 → T025 → T026

Phase 5 (US3):
T027, T028 → T029, T030, T031 → T032 → T033 → T034
```

### Parallel Opportunities

Within Phase 1:
- T002 and T003 can run in parallel (different files)

Within Phase 3 (US1):
- T011 and T012 can run in parallel (tsx and css)

Within Phase 4 (US2):
- T021 and T022 can run in parallel (tsx and css)

Within Phase 5 (US3):
- T027 and T028 can run in parallel (state and component)
- T029, T030, T031 can run in parallel (all CSS)

Within Phase 6:
- T035, T036, T037 can run in parallel (independent concerns)

---

## Parallel Example: User Story 1

```bash
# Launch setup tasks together:
Task: "Create chat page component at src/pages/chat.tsx with basic Layout wrapper"
Task: "Create chat page styles at src/pages/chat.module.css with grid layout"

# Then sequential implementation:
Task: "Add iframe documentation viewer component in src/pages/chat.tsx"
Task: "Add chat panel container with placeholder in src/pages/chat.tsx"
# ... continue sequentially
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational backend (T004-T010)
3. Complete Phase 3: User Story 1 (T011-T020)
4. **STOP and VALIDATE**: Test split-screen chat with auto-navigation
5. Deploy if ready - this is a functional MVP

### Incremental Delivery

1. Setup + Foundational → Backend ready with structured output
2. Add User Story 1 → Split-screen chat works → **MVP Deploy**
3. Add User Story 2 → Manual navigation → Deploy
4. Add User Story 3 → Mobile support → Deploy
5. Polish → Production ready → Final Deploy

---

## Summary

| Phase | Tasks | Parallel Tasks |
|-------|-------|----------------|
| Setup | 3 | 2 |
| Foundational | 7 | 0 |
| User Story 1 (P1) | 10 | 2 |
| User Story 2 (P2) | 6 | 2 |
| User Story 3 (P3) | 8 | 5 |
| Polish | 7 | 3 |
| **Total** | **41** | **14** |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Backend must be deployed to HuggingFace for frontend to work in production
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
