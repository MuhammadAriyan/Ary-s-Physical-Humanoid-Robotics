# Tasks: Docusaurus i18n with Floating Toggle & AI Translation

**Input**: Design documents from `/specs/010-i18n-floating-toggle/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are included in Phase 9 (Polish) as the spec did not explicitly request TDD approach.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `src/` (Docusaurus)
- **Backend**: `backend/` (FastAPI)
- **i18n**: `i18n/ur/` (Urdu translations)
- **Config**: Root level (`docusaurus.config.ts`)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and i18n directory structure

- [ ] T001 Update docusaurus.config.ts with i18n configuration for 'en' and 'ur' locales with RTL direction
- [ ] T002 [P] Create i18n/ur/code.json with empty UI translations object
- [ ] T003 [P] Create i18n/ur/docusaurus-theme-classic/navbar.json with empty object
- [ ] T004 [P] Create i18n/ur/docusaurus-theme-classic/footer.json with empty object
- [ ] T005 [P] Create i18n/ur/docusaurus-plugin-content-docs/current/ directory structure with placeholder files
- [ ] T006 Verify site builds successfully with both locales (npm run build)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 Create database migration backend/db/migrations/003_add_translations.sql with chapter_translations table schema
- [ ] T008 [P] Create backend/models/translation.py with ChapterTranslation model and Pydantic schemas
- [ ] T009 [P] Create src/hooks/useTranslation.ts with TranslationStatus state type and hook skeleton
- [ ] T010 Run database migration to create chapter_translations table

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Switch to Urdu Language (Priority: P1) 🎯 MVP

**Goal**: Enable users to switch from English to Urdu with RTL layout via floating toggle

**Independent Test**: Click floating toggle, select "اردو", verify page reloads with RTL layout

### Implementation for User Story 1

- [ ] T011 [P] [US1] Create src/components/LanguageToggle/styles.module.css with glassmorphism styling (position: fixed, top: 20px, left: 20px, 44x44px)
- [ ] T012 [US1] Create src/components/LanguageToggle/index.tsx with globe icon, dropdown menu showing "English" and "اردو"
- [ ] T013 [US1] Implement locale switching logic in LanguageToggle using useDocusaurusContext and URL navigation
- [ ] T014 [US1] Update src/theme/Root.tsx to render LanguageToggle component
- [ ] T015 [US1] Verify RTL direction is automatically applied when navigating to /ur/* paths

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Switch Back to English (Priority: P1)

**Goal**: Enable users to switch from Urdu back to English with LTR layout

**Independent Test**: From Urdu mode, click toggle, select "English", verify LTR layout returns

### Implementation for User Story 2

- [ ] T016 [US2] Update LanguageToggle in src/components/LanguageToggle/index.tsx to handle switching from Urdu to English
- [ ] T017 [US2] Add visual indicator in toggle showing current language state
- [ ] T018 [US2] Verify LTR direction is applied when navigating from /ur/* to root paths

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Persistent Language Preference (Priority: P2)

**Goal**: Remember user's language preference across sessions using localStorage

**Independent Test**: Select Urdu, close browser, reopen site, verify Urdu is still selected

### Implementation for User Story 3

- [ ] T019 [US3] Add localStorage persistence in src/components/LanguageToggle/index.tsx (save locale on change)
- [ ] T020 [US3] Implement auto-redirect logic in Root.tsx for returning users based on stored preference
- [ ] T021 [US3] Handle edge case when localStorage is unavailable (fallback to URL-based persistence)

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Access Floating Toggle from Any Position (Priority: P2)

**Goal**: Floating toggle remains visible regardless of scroll position

**Independent Test**: Scroll to bottom of long page, verify toggle is still visible and functional

### Implementation for User Story 4

- [ ] T022 [US4] Verify position: fixed styling in src/components/LanguageToggle/styles.module.css maintains visibility during scroll
- [ ] T023 [US4] Add z-index management to prevent overlap with other floating components
- [ ] T024 [US4] Add dropdown positioning logic to prevent dropdown from going off-screen when near page edges
- [ ] T025 [P] [US4] Add dark mode support to src/components/LanguageToggle/styles.module.css using [data-theme='dark'] selector

**Checkpoint**: All user stories should now be independently functional

---

## Phase 7: User Story 5 - Graceful Fallback for Missing Translations (Priority: P3)

**Goal**: Show English content when Urdu translation is not available

**Independent Test**: Navigate to page without Urdu translation while in Urdu mode, verify English content displays

### Implementation for User Story 5

- [ ] T026 [US5] Verify Docusaurus fallback behavior works by default for missing translations
- [ ] T027 [US5] Add visual indicator in TranslateChapter component when showing fallback content
- [ ] T028 [US5] Ensure RTL layout is maintained even when showing English fallback content

---

## Phase 8: User Story 6 & 7 - AI Translation & Save (Priority: P2)

**Goal**: Logged-in users can translate chapters to Urdu via AI and save translations

**Independent Test**: Log in, navigate to chapter, click translate, verify Urdu content appears, save it, verify another user can see it

### Backend Implementation

- [ ] T029 [P] [US6] Create backend/api/translate.py with POST /api/translate endpoint using OpenAI SDK
- [ ] T030 [P] [US7] Create backend/api/translations.py with POST /api/translations (save) and GET /api/translations/:chapterId/:locale (get)
- [ ] T031 [US6] Implement AI translation logic in backend/api/translate.py using OPENAI_BASE_URL environment variable
- [ ] T032 [US6] Add authentication middleware check for translate endpoint (require Bearer token)
- [ ] T033 [US7] Implement save translation logic with upsert for chapter_id/locale uniqueness
- [ ] T034 [US6] Add error handling for AI provider failures (timeout, rate limit, API errors)
- [ ] T035 Register translation routes in backend main application

### Frontend Implementation

- [ ] T036 [P] [US6] Create src/components/TranslateChapter/styles.module.css with button styling, loading state, RTL content display
- [ ] T037 [US6] Create src/components/TranslateChapter/index.tsx with translate button, loading indicator, and translated content display
- [ ] T038 [US6] Implement auth check in TranslateChapter - hide/disable button for non-authenticated users with tooltip
- [ ] T039 [US6] Implement useTranslation hook in src/hooks/useTranslation.ts with translation state machine (untranslated, translating, translated, saved, failed)
- [ ] T040 [US6] Connect TranslateChapter to backend POST /api/translate endpoint
- [ ] T041 [US7] Add "Save Translation" button that appears after successful translation
- [ ] T042 [US7] Connect save functionality to POST /api/translations endpoint
- [ ] T043 [US6] Add retry button for failed translations with error message display
- [ ] T044 [US6] Add RTL layout for translated content display using dir="rtl" attribute

### DocItem Swizzle

- [ ] T045 [US6] Swizzle DocItem/Content component: npx docusaurus swizzle @docusaurus/theme-classic DocItem/Content --wrap
- [ ] T046 [US6] Update swizzled src/theme/DocItem/Content/index.tsx to inject TranslateChapter component at top of content
- [ ] T047 [US6] Pass chapter ID and content props from DocItem to TranslateChapter component

### Saved Translation Loading

- [ ] T048 [US7] Implement saved translation check on page load via GET /api/translations/:chapterId/:locale
- [ ] T049 [US7] Display saved translation content when available instead of original English
- [ ] T050 [US7] Show "Re-translate" option when viewing saved translation

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### RTL CSS Improvements

- [ ] T051 [P] Review and fix any RTL layout issues in src/css/custom.css
- [ ] T052 [P] Add CSS logical properties where needed for proper RTL support
- [ ] T053 [P] Create src/css/rtl.css for RTL-specific overrides if needed

### Mobile Responsiveness

- [ ] T054 [P] Test and fix LanguageToggle on mobile devices (ensure 44x44px tap target)
- [ ] T055 [P] Test and fix TranslateChapter on mobile devices
- [ ] T056 [P] Adjust dropdown positioning for mobile viewports

### Accessibility

- [ ] T057 [P] Add ARIA labels to LanguageToggle (aria-label, aria-expanded, role)
- [ ] T058 [P] Add ARIA labels to TranslateChapter button and loading states
- [ ] T059 [P] Add keyboard navigation support for language dropdown

### Error Handling

- [ ] T060 Implement API timeout handling with user-friendly messages
- [ ] T061 Add rate limit detection and messaging
- [ ] T062 Add graceful degradation when backend is unavailable

### Validation

- [ ] T063 Run full build with both locales: npm run build
- [ ] T064 Validate quickstart.md setup instructions work correctly
- [ ] T065 Manual testing checklist from quickstart.md

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
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May depend on US1 toggle component
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Depends on US1 toggle component
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - Depends on US1 toggle component
- **User Story 5 (P3)**: Can start after Foundational (Phase 2) - Independent of other stories
- **User Story 6 (P2)**: Can start after Foundational (Phase 2) - Depends on backend infrastructure
- **User Story 7 (P2)**: Can start after Foundational (Phase 2) - Depends on US6 translation component

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 6 & 7

```bash
# Launch backend API creation together:
Task: "Create backend/api/translate.py with POST /api/translate endpoint"
Task: "Create backend/api/translations.py with save and get endpoints"

# Launch frontend implementation with backend:
Task: "Create src/components/TranslateChapter/styles.module.css with button styling"
Task: "Create src/components/TranslateChapter/index.tsx with translate button"

# Then sequentially implement functionality:
Task: "Implement AI translation logic in backend/api/translate.py"
Task: "Connect TranslateChapter to backend POST /api/translate endpoint"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: US1 - Switch to Urdu
4. Complete Phase 4: US2 - Switch to English
5. **STOP and VALIDATE**: Test language switching independently
6. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add US1 + US2 → Basic toggle working → Deploy/Demo (MVP!)
3. Add US3 → Persistence working → Deploy/Demo
4. Add US4 → Better UX → Deploy/Demo
5. Add US5 → Fallback handling → Deploy/Demo
6. Add US6 + US7 → AI translation → Deploy/Demo (Full Feature!)
7. Polish → Production ready

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: US1 (Switch to Urdu)
   - Developer B: US2 (Switch to English)
   - Developer C: US3 (Persistence)
   - Developer D: US4 (Floating Toggle UX)
   - Developer E: US5 (Fallback)
3. Then work on US6 & US7 (AI Translation)
4. Finally, polish phase

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify the site builds after each phase to catch issues early
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- AI Translation (US6 & US7) is the largest effort - can be deferred for MVP
- Verify Docusaurus builds after each phase to catch issues early
- Commit after each task or logical group