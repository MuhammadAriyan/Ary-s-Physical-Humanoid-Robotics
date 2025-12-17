# Data Model: Custom Header/Navbar Component

**Feature Branch**: `009-custom-header`
**Date**: 2025-12-17

## Overview

The Header component is primarily a UI component with minimal data requirements. It relies on existing auth state and Docusaurus context rather than managing its own data.

## Entities

### 1. HeaderState (Component State)

Internal state managed by the Header component.

| Field | Type | Description |
|-------|------|-------------|
| mobileMenuOpen | boolean | Whether mobile drawer menu is open |
| userDropdownOpen | boolean | Whether user dropdown is open |
| languageDropdownOpen | boolean | Whether language dropdown is open |

**State Transitions**:
- `mobileMenuOpen`: false → true (hamburger click) → false (close/navigate/backdrop click)
- `userDropdownOpen`: false → true (avatar click) → false (selection/click outside)
- `languageDropdownOpen`: false → true (selector click) → false (selection/click outside)

---

### 2. AuthUser (External - from Better Auth)

User data retrieved from `useAuth()` hook. **Not managed by Header**.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique user identifier |
| email | string | User's email address |
| name | string \| null | User's display name |
| image | string \| null | URL to user's avatar image |
| emailVerified | boolean | Whether email is verified |

**Source**: `src/lib/auth-client.ts` via `useAuth()` hook

---

### 3. Locale (Configuration)

Language/locale configuration for the language selector.

| Field | Type | Description |
|-------|------|-------------|
| code | string | Locale code (e.g., 'en', 'es') |
| label | string | Display name (e.g., 'English') |
| flag | string | Emoji flag (e.g., '🇺🇸') |

**Initial Data**:
```typescript
const LOCALES: Locale[] = [
  { code: 'en', label: 'English', flag: '🇺🇸' },
  // Future locales can be added here
];
```

---

### 4. ExternalLink (Configuration)

External profile links configuration.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique identifier |
| href | string | Full URL |
| label | string | Accessibility label |
| icon | ReactNode | SVG icon component |

**Initial Data**:
```typescript
const EXTERNAL_LINKS: ExternalLink[] = [
  {
    id: 'github',
    href: 'https://github.com/MuhammadAriyan',
    label: 'GitHub Profile',
    icon: <GitHubIcon />
  },
  {
    id: 'linkedin',
    href: 'https://www.linkedin.com/in/muhammad-aryan',
    label: 'LinkedIn Profile',
    icon: <LinkedInIcon />
  },
];
```

---

## Relationships

```
┌─────────────────────────────────────────────────────────────┐
│                         Header                               │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ State: mobileMenuOpen, userDropdownOpen, langDropdown │   │
│  └──────────────────────────────────────────────────────┘   │
│                           │                                  │
│           ┌───────────────┼───────────────┐                 │
│           ▼               ▼               ▼                 │
│    ┌──────────┐   ┌──────────────┐  ┌────────────┐         │
│    │ AuthUser │   │ Locale[]     │  │ ExternalLink│         │
│    │ (external)│   │ (config)     │  │ (config)   │         │
│    └──────────┘   └──────────────┘  └────────────┘         │
│         │                │                  │               │
│         ▼                ▼                  ▼               │
│    ┌──────────┐   ┌──────────────┐  ┌────────────┐         │
│    │UserDropdown│  │LangSelector │  │SocialLinks │         │
│    └──────────┘   └──────────────┘  └────────────┘         │
└─────────────────────────────────────────────────────────────┘
```

---

## Validation Rules

### User Display
- **Name truncation**: If `user.name` or `user.email` exceeds 20 characters, truncate with ellipsis
- **Avatar fallback**: If `user.image` is null, show first character of name or email

### Dropdowns
- **Mutual exclusivity**: Only one dropdown open at a time
- **Mobile menu**: Closes all dropdowns when opened
- **Click outside**: Closes any open dropdown

### Links
- **External links**: Must open in new tab (`target="_blank"`)
- **Security**: Include `rel="noopener noreferrer"` for external links

---

## Data Flow

```
1. Auth State Flow:
   AuthProvider (Root.tsx)
         │
         ▼
   useAuth() hook ───► Header component
         │
         ▼
   Display: SignIn button OR User avatar + dropdown

2. i18n Flow:
   Docusaurus Config (locales)
         │
         ▼
   useActiveLocale() ───► LanguageSelector
         │
         ▼
   Display: Current locale + available locales

3. Navigation Flow:
   User clicks logo
         │
         ▼
   Docusaurus Router ───► Homepage ("/")
```

---

## No Backend API Required

This feature is purely frontend. All data sources are:
- **Auth state**: From existing Better Auth hooks
- **Locale config**: From Docusaurus i18n config
- **External links**: Hardcoded configuration

No new API endpoints or backend modifications are needed.
