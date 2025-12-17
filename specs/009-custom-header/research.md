# Research: Custom Header/Navbar Component

**Feature Branch**: `009-custom-header`
**Date**: 2025-12-17
**Status**: Complete

## Research Tasks

### 1. Auth Integration Pattern

**Decision**: Reuse existing `useAuth` hook and `AuthModal` component from `src/components/Auth/`

**Rationale**:
- Existing auth system is fully functional with Better Auth
- `useAuth` hook provides all needed state: `user`, `isAuthenticated`, `isLoading`, `sessionExpired`, `refetch`
- `AuthModal` already handles signin/signup flows with Google OAuth
- Maintains consistency across the application

**Alternatives Considered**:
- Create new auth components: Rejected - duplicates existing functionality
- Inline auth logic in header: Rejected - violates DRY principle

**Implementation Notes**:
```typescript
// Import pattern
import { useAuth, AuthModal, AuthButton } from '../Auth';

// Usage
const { user, isAuthenticated, isLoading, refetch } = useAuth();
```

---

### 2. Styling System

**Decision**: Use existing CSS variables from `src/styles/` with CSS Modules

**Rationale**:
- Comprehensive design system already exists (glassmorphism.css, luxury.css, typography.css)
- CSS Modules provide scoped styles while allowing use of global CSS variables
- Existing patterns (BEM-like naming, dark mode selectors) ensure consistency

**Key CSS Variables to Use**:

| Purpose | Variable | Value |
|---------|----------|-------|
| Glass background | `--glass-white-heavy` | `rgba(255, 255, 255, 0.95)` |
| Backdrop blur | `--backdrop-blur-md` | `blur(12px)` |
| Border | `--border-light` | `1px solid rgba(0, 0, 0, 0.08)` |
| Shadow | `--shadow-soft` | `0 4px 12px rgba(0, 0, 0, 0.08)` |
| Font display | `--font-display` | `'Playfair Display', Georgia, serif` |
| Font primary | `--font-primary` | `'Inter', -apple-system, sans-serif` |
| Z-index | `--z-fixed` | `1030` |
| Transition | `--transition-normal` | `0.3s ease` |

**Dark Mode Pattern**:
```css
[data-theme='dark'] .header {
  background: var(--glass-black-heavy);
  /* ... other dark mode overrides */
}
```

---

### 3. Docusaurus Integration

**Decision**: Replace content in `src/theme/Navbar/index.js` with custom Header component

**Rationale**:
- Current navbar is already a swizzled wrapper around Docusaurus original
- `custom.css` already hides default Docusaurus navbar (`.navbar { display: none; }`)
- Theme customization via src/theme/ is the established pattern

**Alternatives Considered**:
- Create separate component outside theme: Rejected - wouldn't integrate with Docusaurus lifecycle
- Modify docusaurus.config.ts navbar items: Rejected - limited customization options

**Implementation Notes**:
- Header component will be in `src/components/Header/`
- `src/theme/Navbar/index.js` will import and render the Header
- Must preserve the AuthProvider context wrapping

---

### 4. i18n Structure

**Decision**: Implement language selector UI with Docusaurus i18n hooks, ready for future locales

**Rationale**:
- Docusaurus has built-in i18n support
- Current config: `{ defaultLocale: 'en', locales: ['en'] }`
- Structure allows easy addition of locales without code changes

**Implementation**:
```typescript
// Use Docusaurus context
import { useActiveLocale, useAlternatePageUtils } from '@docusaurus/theme-common';

// Or simpler approach for MVP
const locales = [
  { code: 'en', label: 'English', flag: '🇺🇸' },
  // Future: { code: 'es', label: 'Español', flag: '🇪🇸' },
];
```

---

### 5. SVG Icons

**Decision**: Create inline SVG React components for GitHub and LinkedIn icons

**Rationale**:
- Inline SVGs allow color customization via CSS `currentColor`
- No additional dependencies required
- Consistent with existing patterns (Google icon in AuthModal uses inline SVG)

**Implementation**:
```typescript
// src/components/Header/icons/GitHubIcon.tsx
export const GitHubIcon: React.FC<{ className?: string }> = ({ className }) => (
  <svg className={className} viewBox="0 0 24 24" fill="currentColor">
    {/* GitHub path */}
  </svg>
);
```

---

### 6. Mobile Menu Pattern

**Decision**: Slide-out drawer menu triggered by hamburger icon

**Rationale**:
- Common mobile pattern users are familiar with
- Can reuse glassmorphism styling from existing components
- Allows full-height menu with all navigation items

**Implementation Notes**:
- State: `const [mobileMenuOpen, setMobileMenuOpen] = useState(false)`
- Breakpoint: `768px` (matches existing responsive patterns)
- Animation: Use `--transition-normal` for slide-in effect
- Backdrop: Semi-transparent overlay with click-to-close

---

### 7. User Avatar/Dropdown

**Decision**: Avatar with initial fallback + dropdown menu for authenticated state

**Rationale**:
- Existing AuthButton pattern shows avatar with first character of name/email
- Dropdown provides clean UX for user actions without navigating away

**Implementation**:
```typescript
// Avatar rendering
const getInitial = () => {
  if (user?.name) return user.name.charAt(0).toUpperCase();
  if (user?.email) return user.email.charAt(0).toUpperCase();
  return '?';
};

// Dropdown content
- User name/email (truncated to 20 chars)
- "Sign Out" button
```

---

## Technical Decisions Summary

| Decision | Choice | Key Reason |
|----------|--------|------------|
| Auth | Reuse existing hooks | Consistency, DRY |
| Styling | CSS Modules + CSS vars | Scoped styles, theme integration |
| Integration | Swizzled Navbar | Docusaurus pattern |
| i18n | Docusaurus hooks | Future-proof structure |
| Icons | Inline SVG components | Color customization |
| Mobile | Slide-out drawer | UX familiarity |
| Avatar | Initial fallback | Graceful degradation |

## Constitution Compliance Check

| Requirement | Status | Notes |
|-------------|--------|-------|
| Frontend uses React + TypeScript | ✅ PASS | Header.tsx with TypeScript |
| Matches Docusaurus theme | ✅ PASS | Using existing CSS variables |
| Static deployable | ✅ PASS | No server-side rendering required |
| No vector DB/RAG | ✅ PASS | N/A for UI component |
