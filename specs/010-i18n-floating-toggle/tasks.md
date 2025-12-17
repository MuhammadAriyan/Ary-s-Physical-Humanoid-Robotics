# Tasks: Docusaurus i18n with Floating Toggle & AI Translation

**Input**: Design documents from `/specs/010-i18n-floating-toggle/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api.yaml

**Tests**: Tests are included in Phase 9 (Polish) as the spec did not explicitly request TDD approach.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `src/` (Docusaurus)
- **Backend**: `backend/` (FastAPI)
- **i18n**: `i18n/ur/` (Urdu translations)
- **Config**: Root level (`docusaurus.config.ts`)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and i18n directory structure

- [x] T001 Update docusaurus.config.ts with i18n configuration for 'en' and 'ur' locales with RTL direction
- [x] T002 [P] Create i18n/ur/code.json with empty UI translations object
- [x] T003 [P] Create i18n/ur/docusaurus-theme-classic/navbar.json with empty object
- [x] T004 [P] Create i18n/ur/docusaurus-theme-classic/footer.json with empty object
- [x] T005 [P] Create i18n/ur/docusaurus-plugin-content-docs/current/ directory structure with placeholder files
- [x] T006 Verify site builds successfully with both locales (npm run build)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before user stories can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Create database migration backend/db/migrations/003_add_translations.sql with chapter_translations table schema
- [x] T008 [P] Create backend/models/translation.py with ChapterTranslation model and Pydantic schemas
- [ ] T009 [P] Create src/hooks/useTranslation.ts with TranslationStatus state type and hook skeleton
- [x] T010 Run database migration to create chapter_translations table

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Switch to Urdu Language (Priority: P1)

**Goal**: Enable users to switch from English to Urdu with RTL layout via floating toggle

**Independent Test**: Click floating toggle, select "اردو", verify page reloads with RTL layout

### Implementation for User Story 1

- [x] T011 [P] [US1] Create src/components/LanguageToggle/styles.module.css with glassmorphism styling (position: fixed, top: 20px, left: 20px, 44x44px)
- [x] T012 [US1] Create src/components/LanguageToggle/index.tsx with globe icon, dropdown menu showing "English" and "اردو"
- [x] T013 [US1] Implement locale switching logic in LanguageToggle using useDocusaurusContext and URL navigation
- [x] T014 [US1] Update src/theme/Root.tsx to render LanguageToggle component
- [x] T015 [US1] Verify RTL direction is automatically applied when navigating to /ur/* paths

**Checkpoint**: User Story 1 complete - users can switch to Urdu with RTL layout

---

## Phase 4: User Story 2 - Switch Back to English (Priority: P1)

**Goal**: Enable users to switch from Urdu back to English with LTR layout

**Independent Test**: From Urdu mode, click toggle, select "English", verify LTR layout returns

### Implementation for User Story 2

- [x] T016 [US2] Update LanguageToggle in src/components/LanguageToggle/index.tsx to handle switching from Urdu to English
- [x] T017 [US2] Add visual indicator in toggle showing current language state
- [x] T018 [US2] Verify LTR direction is applied when navigating from /ur/* to root paths

**Checkpoint**: Users can switch between languages in both directions

---

## Phase 5: User Story 3 - Persistent Language Preference (Priority: P2)

**Goal**: Remember user's language preference across sessions using localStorage

**Independent Test**: Select Urdu, close browser, reopen site, verify Urdu is still selected

### Implementation for User Story 3

- [x] T019 [US3] Add localStorage persistence in src/components/LanguageToggle/index.tsx (save locale on change)
- [ ] T020 [US3] Implement auto-redirect logic in Root.tsx for returning users based on stored preference
- [x] T021 [US3] Handle edge case when localStorage is unavailable (fallback to URL-based persistence)

**Checkpoint**: Language preference persists across browser sessions

---

## Phase 6: User Story 4 - Access Floating Toggle from Any Position (Priority: P2)

**Goal**: Floating toggle remains visible regardless of scroll position

**Independent Test**: Scroll to bottom of long page, verify toggle is still visible and functional

### Implementation for User Story 4

- [x] T022 [US4] Verify position: fixed styling in src/components/LanguageToggle/styles.module.css maintains visibility during scroll
- [x] T023 [US4] Add z-index management to prevent overlap with other floating components
- [x] T024 [US4] Add dropdown positioning logic to prevent dropdown from going off-screen when near page edges
- [x] T025 [P] [US4] Add dark mode support to src/components/LanguageToggle/styles.module.css using [data-theme='dark'] selector

**Checkpoint**: Toggle accessible from any scroll position

---

## Phase 7: User Story 5 - Graceful Fallback for Missing Translations (Priority: P3)

**Goal**: Show English content when Urdu translation is not available

**Independent Test**: Navigate to page without Urdu translation while in Urdu mode, verify English content displays

### Implementation for User Story 5

- [x] T026 [US5] Verify Docusaurus fallback behavior works by default for missing translations
- [x] T027 [US5] Add visual indicator in TranslateChapter component when showing fallback content
- [x] T028 [US5] Ensure RTL layout is maintained even when showing English fallback content

**Checkpoint**: Graceful fallback for missing translations working

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

**Checkpoint**: AI translation and save functionality complete

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

```
Phase 1 (Setup)
    │
    ▼
Phase 2 (Foundational) ── BLOCKS ALL USER STORIES
    │
    ├───────────────────────────────────────┐
    │                                       │
    ▼                                       ▼
Phase 3 (US1: Switch to Urdu) ─────► Phase 4 (US2: Switch to English)
    │                                       │
    │                                       │
    ▼                                       │
Phase 5 (US3: Persistence) ◄────────────────┘
    │
    ▼
Phase 6 (US4: Floating Toggle UX)
    │
    ▼
Phase 7 (US5: Fallback)
    │
    ▼
Phase 8 (US6 & US7: AI Translation)
    │
    ▼
Phase 9 (Polish)
```

### User Story Dependencies

- **US1 (P1)**: No dependencies - can start after Phase 2
- **US2 (P1)**: Depends on US1 toggle component existing
- **US3 (P2)**: Depends on US1/US2 toggle component
- **US4 (P2)**: Depends on US1 toggle component styling
- **US5 (P3)**: Can start after Phase 2, independent of other stories
- **US6 (P2)**: Depends on Phase 2 (database, models)
- **US7 (P2)**: Depends on US6 translation component

### Parallel Opportunities

**Within Phase 1 (Setup):**
```
T002, T003, T004, T005 can run in parallel
```

**Within Phase 2 (Foundational):**
```
T008, T009 can run in parallel
```

**Within Phase 8 (AI Translation):**
```
Backend: T029, T030 can run in parallel
Frontend: T036 can run in parallel with backend tasks
```

**Within Phase 9 (Polish):**
```
All tasks marked [P] can run in parallel
```

---

## Parallel Example: Phase 8 Backend

```bash
# Launch backend API creation in parallel:
Task: "Create backend/api/translate.py with POST /api/translate endpoint"
Task: "Create backend/api/translations.py with save and get endpoints"

# Then sequentially:
Task: "Implement AI translation logic"
Task: "Add authentication middleware"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

1. Complete Phase 1: Setup - i18n configuration
2. Complete Phase 2: Foundational - database, models
3. Complete Phase 3: US1 - Switch to Urdu
4. Complete Phase 4: US2 - Switch to English
5. **STOP and VALIDATE**: Test language switching independently
6. Deploy/demo floating toggle MVP

### Incremental Delivery

1. **Setup + Foundational** → Foundation ready
2. **Add US1 + US2** → Basic toggle working → Deploy/Demo (MVP!)
3. **Add US3** → Persistence working → Deploy/Demo
4. **Add US4** → Better UX → Deploy/Demo
5. **Add US5** → Fallback handling → Deploy/Demo
6. **Add US6 + US7** → AI translation → Deploy/Demo (Full Feature!)
7. **Polish** → Production ready

### Suggested MVP Scope

**Minimum Viable Product**: Phase 1-4 (Setup + Foundational + US1 + US2)
- Users can switch between English and Urdu
- RTL layout works correctly
- ~15 tasks

**Full Feature**: All phases through Phase 8
- AI translation for logged-in users
- Saved translations persist
- ~50 tasks

---

## Summary

| Phase | User Story | Priority | Task Count |
|-------|------------|----------|------------|
| 1 | Setup | - | 6 |
| 2 | Foundational | - | 4 |
| 3 | US1: Switch to Urdu | P1 | 5 |
| 4 | US2: Switch to English | P1 | 3 |
| 5 | US3: Persistence | P2 | 3 |
| 6 | US4: Floating Toggle UX | P2 | 4 |
| 7 | US5: Fallback | P3 | 3 |
| 8 | US6 & US7: AI Translation | P2 | 22 |
| 9 | Polish | - | 15 |
| **Total** | | | **65** |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently testable after completion
- AI Translation (US6 & US7) is the largest effort - can be deferred for MVP
- Verify Docusaurus builds after each phase to catch issues early
- Commit after each task or logical group
