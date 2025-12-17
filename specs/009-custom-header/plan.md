# Implementation Plan: Custom Header/Navbar Component

**Branch**: `009-custom-header` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/009-custom-header/spec.md`

**Note**: Implementation should use the **front-end-engineer-emma** subagent for all React/TypeScript/CSS work.

## Summary

Create a custom header/navbar component for the Docusaurus site featuring site branding with Playfair Display font, Better Auth integration for sign-in/sign-out, GitHub/LinkedIn social links, language selector dropdown (i18n ready), and responsive mobile hamburger menu. The component uses the existing glassmorphism design system with CSS Modules.

## Technical Context

**Language/Version**: TypeScript 5.6 + React 19
**Primary Dependencies**: React, better-auth, clsx, Docusaurus 3.9.2
**Storage**: N/A (UI component only, uses existing auth state)
**Testing**: Manual testing + Playwright (existing)
**Target Platform**: Web (GitHub Pages static deployment)
**Project Type**: Web application (Docusaurus frontend)
**Performance Goals**: < 1s header render, no layout shift during navigation
**Constraints**: Static deployment only, no SSR; must integrate with existing auth
**Scale/Scope**: Single header component with ~5 sub-components

## Constitution Check

*GATE: PASSED*

| Requirement | Status | Notes |
|-------------|--------|-------|
| Agent named "Fubuni" | N/A | Header component, not agent |
| OpenAI-compatible providers only | N/A | No AI integration in header |
| Backend in Python 3.11 + OpenAI SDK | N/A | Frontend-only feature |
| Frontend matches Docusaurus theme | ✅ PASS | Uses existing CSS variables |
| Static deployable | ✅ PASS | No server required |
| No vector DB/crawling/embeddings | ✅ PASS | UI component only |
| Architecture supports later docs ingestion | N/A | Not applicable to header |

## Project Structure

### Documentation (this feature)

```text
specs/009-custom-header/
├── plan.md              # This file
├── research.md          # Phase 0 output ✅
├── data-model.md        # Phase 1 output ✅
├── quickstart.md        # Phase 1 output ✅
├── contracts/           # Phase 1 output ✅
│   └── component-interfaces.ts
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code (to be created)

```text
src/
├── components/
│   ├── Auth/                    # Existing - reuse
│   │   ├── AuthButton.tsx
│   │   ├── AuthModal.tsx
│   │   ├── AuthProvider.tsx
│   │   └── index.ts
│   └── Header/                  # NEW - to be created
│       ├── Header.tsx           # Main header component
│       ├── Header.module.css    # Header styles (glassmorphism)
│       ├── LanguageSelector.tsx # Language dropdown
│       ├── UserMenu.tsx         # Authenticated user dropdown
│       ├── MobileMenu.tsx       # Mobile drawer menu
│       ├── SocialLinks.tsx      # GitHub/LinkedIn icons
│       ├── icons/
│       │   ├── GitHubIcon.tsx
│       │   ├── LinkedInIcon.tsx
│       │   ├── HamburgerIcon.tsx
│       │   ├── CloseIcon.tsx
│       │   └── ChevronIcon.tsx
│       └── index.ts             # Exports
├── theme/
│   └── Navbar/
│       └── index.js             # Updated to use Header
└── lib/
    └── auth-client.ts           # Existing - reuse
```

**Structure Decision**: Web application structure using existing src/components/ and src/theme/ directories. Header is a new component folder within the established component hierarchy.

## Complexity Tracking

No constitution violations - feature is straightforward UI component.

## Implementation Phases

### Phase 1: Core Header Structure (P1 - Branding)

**Goal**: Render header with logo that navigates to homepage

**Tasks**:
1. Create `src/components/Header/Header.tsx` with basic structure
2. Create `src/components/Header/Header.module.css` with glassmorphism styling
3. Create `src/components/Header/index.ts` exports
4. Update `src/theme/Navbar/index.js` to render Header
5. Implement logo with Playfair Display font linking to `/`

**Acceptance**: Logo visible on all pages, clicking navigates to homepage

---

### Phase 2: Auth Integration (P1 - Authentication)

**Goal**: Show sign in/sign out functionality in header

**Tasks**:
1. Create `src/components/Header/UserMenu.tsx` for authenticated state
2. Import and use `useAuth` hook from existing auth components
3. Show "Sign In" button when not authenticated
4. Show user avatar + dropdown when authenticated
5. Integrate `AuthModal` for sign-in flow
6. Implement sign-out in dropdown

**Acceptance**: Can sign in/out from header without page navigation

---

### Phase 3: Social Links (P2 - External Profiles)

**Goal**: Display GitHub and LinkedIn icons with correct links

**Tasks**:
1. Create `src/components/Header/icons/GitHubIcon.tsx` (inline SVG)
2. Create `src/components/Header/icons/LinkedInIcon.tsx` (inline SVG)
3. Create `src/components/Header/SocialLinks.tsx` component
4. Add hover effects and tooltips
5. Ensure links open in new tab with security attributes

**Acceptance**: Icons visible, clicking opens correct URL in new tab

---

### Phase 4: Mobile Responsiveness (P2 - Mobile)

**Goal**: Hamburger menu with all navigation on mobile

**Tasks**:
1. Create `src/components/Header/icons/HamburgerIcon.tsx`
2. Create `src/components/Header/icons/CloseIcon.tsx`
3. Create `src/components/Header/MobileMenu.tsx` drawer component
4. Add CSS media query for 768px breakpoint
5. Hide desktop nav, show hamburger on mobile
6. Implement slide-out drawer with backdrop

**Acceptance**: All navigation accessible via hamburger menu on mobile

---

### Phase 5: Language Selector (P3 - i18n)

**Goal**: Language dropdown UI ready for future locales

**Tasks**:
1. Create `src/components/Header/icons/ChevronIcon.tsx`
2. Create `src/components/Header/LanguageSelector.tsx`
3. Show current locale (English) with dropdown
4. Structure for Docusaurus i18n integration
5. Add to mobile menu

**Acceptance**: Dropdown shows current language, ready for locale expansion

---

### Phase 6: Polish & Accessibility

**Goal**: Dark mode, animations, keyboard navigation, ARIA

**Tasks**:
1. Add dark mode styles via `[data-theme='dark']` selector
2. Add hover animations using CSS variables
3. Implement keyboard navigation (Tab, Enter, Escape)
4. Add ARIA labels to all interactive elements
5. Test with screen reader
6. Fix any layout shift issues

**Acceptance**: All SC criteria met (accessibility audit pass)

## Dependencies

### External Dependencies (already in package.json)
- `better-auth`: ^1.4.7 (existing)
- `clsx`: ^2.0.0 (existing)
- `react`: ^19.0.0 (existing)

### Internal Dependencies
- `src/components/Auth/` - AuthModal, AuthProvider, useAuth hook
- `src/styles/*.css` - CSS variables for glassmorphism, typography
- `src/lib/auth-client.ts` - signOut function

## Risk Analysis

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Auth integration issues | Low | Medium | Reuse existing working AuthButton pattern |
| CSS variable conflicts | Low | Low | Use CSS Modules for scoping |
| Mobile menu z-index issues | Medium | Low | Use established z-index scale |
| Dark mode inconsistency | Low | Medium | Test both modes early |

## Next Steps

Run `/sp.tasks` to generate detailed task breakdown from this plan.

**Implementation Note**: Use the **front-end-engineer-emma** subagent for all React/TypeScript/CSS implementation work.
