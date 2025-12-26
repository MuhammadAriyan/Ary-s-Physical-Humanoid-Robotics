---
description: "Task list for Urdu Documentation Translation feature"
---

# Tasks: Urdu Documentation Translation

**Input**: Design documents from `/specs/014-urdu-docs-translation/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: N/A - Manual verification only for this documentation feature

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization - Create language toggle infrastructure

- [ ] T001 Create LanguageContext.tsx with localStorage persistence in src/components/Language/LanguageContext.tsx
- [ ] T002 [P] Create LanguageToggle.tsx component in src/components/Language/LanguageToggle.tsx

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Configure Docusaurus for Urdu locale - CRITICAL before user stories

- [ ] T003 Update docusaurus.config.ts to add 'ur' locale with RTL direction

**Checkpoint**: Foundation ready - language toggle infrastructure and Urdu locale configured

---

## Phase 3: User Story 1 - Language Toggle Access (Priority: P1) 🎯 MVP

**Goal**: Users can click Urdu toggle button and navigate to /ur/docs/

**Independent Test**: Visit docs page, click Urdu toggle, verify navigation to /ur/docs/ URL

### Implementation for User Story 1

- [ ] T004 [P] [US1] Add LanguageToggle to index.tsx at end of page in src/pages/index.tsx
- [ ] T005 [US1] Add LanguageToggle to docs.tsx at end of page in src/pages/docs.tsx (depends on T004 for context usage)
- [ ] T006 [US1] Verify toggle navigates to /ur/docs/ when clicked

**Checkpoint**: User Story 1 complete - toggle button works on both pages

---

## Phase 4: User Story 2 - Urdu Content Display (Priority: P1)

**Goal**: Users can read documentation in Urdu with proper formatting

**Independent Test**: Visit /ur/docs/ and verify content is in Urdu with English terms in parentheses

### Implementation for User Story 2

- [ ] T007 [P] [US2] Create docs/ur/ folder structure mirroring docs/
- [ ] T008 [P] [US2] Translate docs/part-1-foundations/01-introduction-to-physical-ai.md to Urdu
- [ ] T009 [P] [US2] Translate docs/part-1-foundations/02-course-overview.md to Urdu
- [ ] T010 [US2] Translate docs/part-2-ros2/02-ros2-fundamentals.md to Urdu (keep code in English, add Urdu comments)

**Checkpoint**: User Story 2 partial - Part 1 and Part 2 ROS2 translated

---

## Phase 5: User Story 3 - Technical Term Clarity (Priority: P2)

**Goal**: Key technical terms appear as "Urdu term (English term)" format

**Independent Test**: Check that terms like Physical AI, ROS 2, Gazebo appear as Urdu (English) format

### Implementation for User Story 3

- [ ] T011 [P] [US3] Translate docs/part-2-ros2/02a-week-3-5-overview.md to Urdu with key terms format
- [ ] T012 [P] [US3] Translate docs/part-3-simulation/03-simulation-setup.md to Urdu with key terms format
- [ ] T013 [P] [US3] Translate docs/part-3-simulation/03a-week-6-7-overview.md to Urdu with key terms format
- [ ] T014 [US3] Translate docs/part-4-isaac/04-isaac-overview.md to Urdu with key terms format

**Checkpoint**: User Story 3 complete - key terms follow "Urdu (English)" format

---

## Phase 6: User Story 4 - Full Documentation Coverage (Priority: P2)

**Goal**: All 17 documentation files are translated to Urdu

**Independent Test**: Verify all 17 files exist in docs/ur/ with correct folder structure

### Implementation for User Story 4

- [ ] T015 [P] [US4] Translate docs/part-4-isaac/04a-week-8-10-overview.md to Urdu
- [ ] T016 [P] [US4] Translate docs/part-5-humanoid/05-humanoid-development.md to Urdu
- [ ] T017 [P] [US4] Translate docs/part-5-humanoid/05a-week-11-13-overview.md to Urdu
- [ ] T018 [P] [US4] Translate docs/part-6-conversational/06-conversational-ai.md to Urdu
- [ ] T019 [US4] Translate docs/part-6-conversational/06a-week-14-overview.md to Urdu (depends on T018 for flow)

**Checkpoint**: User Story 4 partial - More files translated

---

## Phase 7: Complete Documentation Coverage

**Purpose**: Finish remaining appendices for full coverage

- [ ] T020 [P] Translate docs/appendix/A-hardware-specifications.md to Urdu
- [ ] T021 [P] Translate docs/appendix/B-simulation-setup.md to Urdu
- [ ] T022 [P] Translate docs/appendix/C-community-resources.md to Urdu
- [ ] T023 [US4] Translate docs/appendix/D-assessment-rubrics.md to Urdu
- [ ] T024 [US4] Translate docs/appendix/A-unitree-robots.md to Urdu

**Checkpoint**: All 17 files translated - User Story 4 complete

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final verification and cleanup

- [ ] T025 [P] Verify all 17 files exist in docs/ur/ with correct folder structure
- [ ] T026 [P] Verify all code blocks have Urdu comments above and below
- [ ] T027 Verify key terms follow "Urdu (English)" format throughout
- [ ] T028 [P] Manual testing: Toggle between English and Urdu docs
- [ ] T029 Run npm build to verify static site generation works
- [ ] T030 Commit and push all changes

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if capacity allows)
  - Or sequentially in priority order (US1 → US2 → US3 → US4)
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Depends on US1 toggle working
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Can work in parallel with US2
- **User Story 4 (P2)**: Depends on US3 for key term consistency

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T001, T002)
- Once Foundational completes, User Stories 2 and 3 can start in parallel
- Within each user story, files marked [P] can be translated in parallel

### Parallel Example: User Story 2

```bash
# Launch multiple translations in parallel:
Task: "Translate docs/part-1-foundations/01-introduction-to-physical-ai.md to Urdu"
Task: "Translate docs/part-1-foundations/02-course-overview.md to Urdu"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001, T002)
2. Complete Phase 2: Foundational (T003)
3. Complete Phase 3: User Story 1 (T004, T005, T006)
4. **STOP and VALIDATE**: Test toggle works on both pages
5. Deploy/demo MVP!

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test → Deploy/Demo (MVP!)
3. Add User Story 2 → Test → Deploy/Demo
4. Add User Story 3 → Test → Deploy/Demo
5. Add User Story 4 → Test → Deploy/Demo

---

## Summary

| Metric | Count |
|--------|-------|
| Total Tasks | 30 |
| Setup Tasks | 2 |
| Foundational Tasks | 1 |
| User Story 1 Tasks | 3 |
| User Story 2 Tasks | 4 |
| User Story 3 Tasks | 4 |
| User Story 4 Tasks | 5 |
| Documentation Tasks | 5 |
| Polish Tasks | 6 |

**MVP Scope**: Phases 1-3 (Tasks T001-T006) - Language toggle infrastructure and button on pages

**Parallel Opportunities**: 14 tasks marked [P] can be executed in parallel
