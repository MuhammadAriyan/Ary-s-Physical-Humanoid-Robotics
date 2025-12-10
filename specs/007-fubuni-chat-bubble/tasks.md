---
description: "Task list template for feature implementation"
---

# Tasks: Fubuni Chat Bubble

**Input**: Design documents from `/specs/007-fubuni-chat-bubble/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - Entities from data-model.md
  - Endpoints from contracts/

  Tasks MUST be organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure per implementation plan
- [X] T002 Create frontend directory structure per implementation plan
- [X] T003 [P] Initialize backend requirements.txt with FastAPI, agents, asyncpg, sqlmodel
- [ ] T004 [P] Initialize frontend package.json with Docusaurus dependencies
- [X] T005 Create backend configuration files in backend/app/config/

---
## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Setup database models for ChatSession and ChatMessage in backend/app/models/
- [X] T007 [P] Create database configuration and connection setup in backend/app/config/database.py
- [X] T008 Create Pydantic models for API requests/responses in backend/app/api/models.py
- [X] T009 Setup Fubuni agent with custom agents library in backend/app/agents/fubuni_agent.py
- [X] T010 Create SSE streaming utilities in backend/app/utils/streaming.py
- [X] T011 Setup FastAPI application with CORS in backend/app/main.py
- [X] T012 Create base CSS styles using Infima classes in src/components/FubuniChat/styles.module.css

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---
## Phase 3: User Story 1 - Instant Access to Fubuni (Priority: P1) 🎯 MVP

**Goal**: Enable users to see floating bubble, click it to open chat drawer, and send/receive messages

**Independent Test**: A user can see the floating bubble on any documentation page, click it, and immediately start a conversation with Fubuni without page reload or navigation

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**
- [X] T013 [P] [US1] Contract test for /api/chat endpoint in backend/tests/test_chat.py
- [X] T014 [P] [US1] Integration test for chat functionality in backend/tests/test_chat.py

### Implementation for User Story 1

- [X] T015 [P] [US1] Create FubuniBubble component in src/components/FubuniChat/FubuniBubble.tsx
- [X] T016 [P] [US1] Create ChatDrawer component in src/components/FubuniChat/ChatDrawer.tsx
- [X] T017 [US1] Create ChatMessage component in src/components/FubuniChat/ChatMessage.tsx
- [X] T018 [US1] Create ChatInput component with streaming in src/components/FubuniChat/ChatInput.tsx
- [X] T019 [US1] Create main FubuniChat component in src/components/FubuniChat/FubuniChat.tsx
- [X] T020 [US1] Implement chat API endpoint with SSE streaming in backend/app/api/chat.py
- [X] T021 [US1] Implement chat session creation and message storage in backend/app/api/chat.py
- [X] T022 [US1] Create FubuniChatInjector swizzled component in src/theme/FubuniChatInjector/index.js
- [X] T023 [US1] Integrate frontend components with backend API in FubuniChat.tsx
- [X] T024 [US1] Add pulse animation to bubble in styles.module.css

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---
## Phase 4: User Story 2 - Full-Screen Chat Experience (Priority: P2)

**Goal**: Provide immersive chat experience with full-screen modal and minimize functionality

**Independent Test**: A user can expand the chat to full-screen mode and have a better experience for longer conversations, then return to the compact drawer when needed

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [X] T025 [P] [US2] Contract test for expand/minimize functionality in backend/tests/test_chat.py
- [X] T026 [P] [US2] Integration test for full-screen mode in frontend tests

### Implementation for User Story 2

- [X] T027 [P] [US2] Create ChatModal component in src/components/FubuniChat/ChatModal.tsx
- [X] T028 [US2] Add expand/minimize button functionality in ChatDrawer.tsx
- [X] T029 [US2] Implement expand to full-screen logic in FubuniChat.tsx
- [X] T030 [US2] Implement minimize from full-screen logic in ChatModal.tsx
- [X] T031 [US2] Add full-screen styles in styles.module.css

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---
## Phase 5: User Story 3 - Theme Consistency (Priority: P3)

**Goal**: Ensure chat interface seamlessly integrates with Docusaurus theme in both light and dark modes

**Independent Test**: The chat interface visually matches the Docusaurus theme in both light and dark modes, using the same colors, fonts, and styling patterns

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [X] T032 [P] [US3] Contract test for theme consistency in frontend tests
- [X] T033 [P] [US3] Integration test for dark/light mode in frontend tests

### Implementation for User Story 3

- [X] T034 [P] [US3] Update styles.module.css to use Infima CSS variables for theme consistency
- [X] T035 [US3] Implement dark/light mode detection in FubuniChat.tsx
- [X] T036 [US3] Add theme-aware styling for all chat components
- [X] T037 [US3] Test theme consistency across all components

**Checkpoint**: All user stories should now be independently functional

---
[Add more user stories as needed, following the same pattern]

---
## Phase 6: Additional Features (Priority: P4)

**Goal**: Implement chat history and session management features

### Tests for Additional Features (OPTIONAL - only if tests requested) ⚠️

- [X] T038 [P] [US4] Contract test for /api/chat/history endpoint in backend/tests/test_chat.py
- [X] T039 [P] [US4] Contract test for /api/chat/sessions endpoint in backend/tests/test_chat.py

### Implementation for Additional Features

- [X] T040 [P] [US4] Implement chat history endpoint in backend/app/api/chat.py
- [X] T041 [US4] Implement sessions endpoint in backend/app/api/chat.py
- [X] T042 [US4] Create session history UI component in src/components/FubuniChat/ChatHistory.tsx

**Checkpoint**: All core features including history management are functional

---
## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T043 [P] Documentation updates in docs/
- [X] T044 Code cleanup and refactoring
- [X] T045 Performance optimization across all stories
- [X] T046 [P] Additional unit tests (if requested) in backend/tests/
- [X] T047 Security hardening
- [X] T048 Run quickstart.md validation

---
## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---
## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for /api/chat endpoint in backend/tests/test_chat.py"
Task: "Integration test for chat functionality in backend/tests/test_chat.py"

# Launch all components for User Story 1 together:
Task: "Create FubuniBubble component in src/components/FubuniChat/FubuniBubble.tsx"
Task: "Create ChatDrawer component in src/components/FubuniChat/ChatDrawer.tsx"
Task: "Create ChatMessage component in src/components/FubuniChat/ChatMessage.tsx"
Task: "Create ChatInput component with streaming in src/components/FubuniChat/ChatInput.tsx"
```

---
## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---
## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence