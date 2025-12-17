# Tasks: Custom Header/Navbar Component

**Input**: Design documents from `/specs/009-custom-header/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/component-interfaces.ts, research.md

**Tests**: Not explicitly requested - manual testing via quickstart.md

**Organization**: Tasks grouped by user story for independent implementation. Use **front-end-engineer-emma** subagent for all implementation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1, US2, US3, US4, US5)
- Exact file paths included

## Path Conventions

- Frontend: `src/components/`, `src/theme/`
- Styles: `src/components/Header/*.module.css`
- Icons: `src/components/Header/icons/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create Header component folder structure and base files

- [x] T001 Create directory structure `src/components/Header/` and `src/components/Header/icons/`
- [x] T002 [P] Create component exports in `src/components/Header/index.ts`
- [x] T003 [P] Create base Header.module.css with CSS variable imports in `src/components/Header/Header.module.css`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core Header shell that all user stories build upon

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Create base Header component structure in `src/components/Header/Header.tsx`
- [x] T005 Add glassmorphism header styles (fixed position, 80px height, backdrop-filter) in `src/components/Header/Header.module.css`
- [x] T006 Update `src/theme/Navbar/index.js` to render Header component instead of original navbar
- [x] T007 Add dark mode styles via `[data-theme='dark']` selector in `src/components/Header/Header.module.css`
- [x] T008 Verify header renders on all pages (homepage, docs, blog)

**Checkpoint**: Empty header shell visible on all pages with glassmorphism styling

---

## Phase 3: User Story 1 - Site Branding (Priority: P1) 🎯 MVP

**Goal**: Display site logo/name that navigates to homepage

**Independent Test**: Load any page → see "Ary's Physical & Humanoid Robotics" in Playfair Display → click → navigate to homepage

### Implementation for User Story 1

- [x] T009 [US1] Add logo/site name element with Playfair Display font in `src/components/Header/Header.tsx`
- [x] T010 [US1] Style logo with `--font-display` variable and link styling in `src/components/Header/Header.module.css`
- [x] T011 [US1] Implement homepage navigation on logo click using Docusaurus Link in `src/components/Header/Header.tsx`
- [x] T012 [US1] Add logo hover animation (subtle scale/opacity) in `src/components/Header/Header.module.css`
- [x] T013 [US1] Ensure logo visible and clickable on mobile viewport in `src/components/Header/Header.module.css`

**Checkpoint**: User Story 1 complete - logo displays and navigates to homepage

---

## Phase 4: User Story 2 - Authentication (Priority: P1)

**Goal**: Sign in/out functionality directly from header

**Independent Test**: Click "Sign In" → AuthModal opens → sign in → avatar shows → dropdown → sign out → "Sign In" returns

### Implementation for User Story 2

- [x] T014 [P] [US2] Create UserMenu component shell in `src/components/Header/UserMenu.tsx`
- [x] T015 [P] [US2] Create UserMenu styles in `src/components/Header/UserMenu.module.css`
- [x] T016 [US2] Import useAuth hook and AuthModal from existing auth components in `src/components/Header/Header.tsx`
- [x] T017 [US2] Implement "Sign In" button when not authenticated in `src/components/Header/Header.tsx`
- [x] T018 [US2] Add AuthModal integration (open on Sign In click) in `src/components/Header/Header.tsx`
- [x] T019 [US2] Implement user avatar with initial fallback in `src/components/Header/UserMenu.tsx`
- [x] T020 [US2] Create user dropdown menu (name display, sign out) in `src/components/Header/UserMenu.tsx`
- [x] T021 [US2] Add click-outside handler to close dropdown in `src/components/Header/UserMenu.tsx`
- [x] T022 [US2] Implement sign out functionality using signOut from auth-client in `src/components/Header/UserMenu.tsx`
- [x] T023 [US2] Add loading state indicator during auth operations in `src/components/Header/UserMenu.tsx`
- [x] T024 [US2] Truncate long usernames (max 20 chars with ellipsis) in `src/components/Header/UserMenu.tsx`

**Checkpoint**: User Story 2 complete - full auth flow works from header

---

## Phase 5: User Story 3 - External Profiles (Priority: P2)

**Goal**: GitHub and LinkedIn icons linking to external profiles

**Independent Test**: See GitHub/LinkedIn icons → click → correct URL opens in new tab

### Implementation for User Story 3

- [x] T025 [P] [US3] Create GitHubIcon SVG component in `src/components/Header/icons/GitHubIcon.tsx`
- [x] T026 [P] [US3] Create LinkedInIcon SVG component in `src/components/Header/icons/LinkedInIcon.tsx`
- [x] T027 [P] [US3] Create icon index exports in `src/components/Header/icons/index.ts`
- [x] T028 [US3] Create SocialLinks component in `src/components/Header/SocialLinks.tsx`
- [x] T029 [US3] Style social links with hover effects in `src/components/Header/SocialLinks.module.css`
- [x] T030 [US3] Configure GitHub link (https://github.com/MuhammadAriyan) in `src/components/Header/SocialLinks.tsx`
- [x] T031 [US3] Configure LinkedIn link (https://www.linkedin.com/in/muhammad-aryan) in `src/components/Header/SocialLinks.tsx`
- [x] T032 [US3] Add target="_blank" and rel="noopener noreferrer" for security in `src/components/Header/SocialLinks.tsx`
- [x] T033 [US3] Add tooltips on hover in `src/components/Header/SocialLinks.module.css`
- [x] T034 [US3] Integrate SocialLinks into Header in `src/components/Header/Header.tsx`

**Checkpoint**: User Story 3 complete - social links work correctly

---

## Phase 6: User Story 5 - Mobile Navigation (Priority: P2)

**Goal**: Hamburger menu with all navigation on mobile viewports

**Independent Test**: Resize to < 768px → see hamburger icon → tap → drawer opens with all nav items → close works

### Implementation for User Story 5

- [x] T035 [P] [US5] Create HamburgerIcon SVG component in `src/components/Header/icons/HamburgerIcon.tsx`
- [x] T036 [P] [US5] Create CloseIcon SVG component in `src/components/Header/icons/CloseIcon.tsx`
- [x] T037 [US5] Create MobileMenu drawer component in `src/components/Header/MobileMenu.tsx`
- [x] T038 [US5] Style MobileMenu with slide-in animation in `src/components/Header/MobileMenu.module.css`
- [x] T039 [US5] Add backdrop overlay with click-to-close in `src/components/Header/MobileMenu.tsx`
- [x] T040 [US5] Add CSS media query for 768px breakpoint in `src/components/Header/Header.module.css`
- [x] T041 [US5] Hide desktop nav and show hamburger on mobile in `src/components/Header/Header.module.css`
- [x] T042 [US5] Include all nav items in mobile menu (auth, social, language) in `src/components/Header/MobileMenu.tsx`
- [x] T043 [US5] Integrate MobileMenu into Header with open/close state in `src/components/Header/Header.tsx`
- [x] T044 [US5] Close mobile menu on navigation/link click in `src/components/Header/MobileMenu.tsx`

**Checkpoint**: User Story 5 complete - mobile navigation fully functional

---

## Phase 7: User Story 4 - Language Selector (Priority: P3)

**Goal**: Language dropdown UI ready for future i18n expansion

**Independent Test**: See language selector showing "English" → click → dropdown opens → structure ready for more locales

### Implementation for User Story 4

- [x] T045 [P] [US4] Create ChevronIcon SVG component in `src/components/Header/icons/ChevronIcon.tsx`
- [x] T046 [US4] Create LanguageSelector component in `src/components/Header/LanguageSelector.tsx`
- [x] T047 [US4] Style LanguageSelector dropdown in `src/components/Header/LanguageSelector.module.css`
- [x] T048 [US4] Configure DEFAULT_LOCALES array (English only for now) in `src/components/Header/LanguageSelector.tsx`
- [x] T049 [US4] Implement dropdown open/close with click-outside handler in `src/components/Header/LanguageSelector.tsx`
- [x] T050 [US4] Add locale change handler (structure for Docusaurus i18n) in `src/components/Header/LanguageSelector.tsx`
- [x] T051 [US4] Integrate LanguageSelector into Header in `src/components/Header/Header.tsx`
- [x] T052 [US4] Add LanguageSelector to MobileMenu in `src/components/Header/MobileMenu.tsx`

**Checkpoint**: User Story 4 complete - language selector UI ready

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Accessibility, animations, and final polish

- [x] T053 [P] Add ARIA labels to all interactive elements in all Header components
- [x] T054 [P] Implement keyboard navigation (Tab order, Enter to activate, Escape to close dropdowns) in `src/components/Header/Header.tsx`
- [x] T055 Add visible focus states for keyboard navigation in `src/components/Header/Header.module.css`
- [x] T056 Add smooth hover animations using CSS variables in all `*.module.css` files
- [x] T057 Ensure mutual exclusivity - only one dropdown open at a time in `src/components/Header/Header.tsx`
- [x] T058 Test dark mode styling across all components
- [x] T059 Test viewport responsiveness (320px - 2560px)
- [ ] T060 Run quickstart.md manual testing checklist
- [x] T061 Fix any layout shift issues during page navigation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 - BLOCKS all user stories
- **Phase 3 (US1)**: Depends on Phase 2 - MVP milestone
- **Phase 4 (US2)**: Depends on Phase 2 - can parallel with US1
- **Phase 5 (US3)**: Depends on Phase 2 - can parallel with US1/US2
- **Phase 6 (US5)**: Depends on Phase 2 + needs US2/US3/US4 components
- **Phase 7 (US4)**: Depends on Phase 2 - can parallel with others
- **Phase 8 (Polish)**: Depends on all user stories complete

### User Story Dependencies

| Story | Depends On | Can Parallel With |
|-------|------------|-------------------|
| US1 (Branding) | Phase 2 only | US2, US3, US4 |
| US2 (Auth) | Phase 2 only | US1, US3, US4 |
| US3 (Social) | Phase 2 only | US1, US2, US4 |
| US4 (Language) | Phase 2 only | US1, US2, US3 |
| US5 (Mobile) | Phase 2 + US2, US3, US4 content | - |

### Within Each User Story

1. Create component files first (parallelizable)
2. Implement core functionality
3. Add styling
4. Integrate into Header
5. Test independently

---

## Parallel Execution Examples

### Phase 1 Setup (all parallel)
```
T001 Create directory structure
T002 [P] Create index.ts exports
T003 [P] Create base CSS file
```

### User Story 3 Icons (all parallel)
```
T025 [P] [US3] GitHubIcon.tsx
T026 [P] [US3] LinkedInIcon.tsx
T027 [P] [US3] icons/index.ts
```

### Multiple User Stories (after Phase 2)
```
Developer A: US1 (T009-T013)
Developer B: US2 (T014-T024)
Developer C: US3 (T025-T034)
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (header shell)
3. Complete Phase 3: User Story 1 (logo/branding)
4. Complete Phase 4: User Story 2 (auth)
5. **STOP and VALIDATE**: Test branding + auth independently
6. Deploy/demo MVP

### Incremental Delivery

1. Setup + Foundational → Header shell ready
2. Add US1 (Branding) → Deploy (basic header)
3. Add US2 (Auth) → Deploy (header with auth)
4. Add US3 (Social Links) → Deploy
5. Add US5 (Mobile) → Deploy (responsive)
6. Add US4 (Language) → Deploy (i18n ready)
7. Polish → Final release

---

## Summary

| Phase | Tasks | Parallel Tasks | Story |
|-------|-------|----------------|-------|
| Setup | 3 | 2 | - |
| Foundational | 5 | 0 | - |
| US1 Branding | 5 | 0 | P1 |
| US2 Auth | 11 | 2 | P1 |
| US3 Social | 10 | 3 | P2 |
| US5 Mobile | 10 | 2 | P2 |
| US4 Language | 8 | 1 | P3 |
| Polish | 9 | 2 | - |
| **Total** | **61** | **12** | - |

---

## Notes

- Use **front-end-engineer-emma** subagent for all implementation tasks
- [P] tasks can run in parallel (different files)
- [USx] label maps task to user story
- Verify each user story independently before moving to next
- Commit after each task or logical group
- Test dark mode early (Phase 2)
