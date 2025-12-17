# Feature Specification: Docusaurus i18n with Floating Language Toggle

**Feature Branch**: `010-i18n-floating-toggle`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Add internationalization (i18n) support to the Docusaurus site with Urdu and English languages, using a floating language toggle button instead of navbar. Logged-in users can translate chapter content to Urdu via AI-powered auto-translation button at the start of each chapter."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Switch to Urdu Language (Priority: P1)

An Urdu-speaking visitor arrives at the documentation site and wants to read content in their native language. They locate the floating language toggle button and switch to Urdu, causing the entire site to display in right-to-left (RTL) format with Urdu text.

**Why this priority**: This is the core value proposition - enabling Urdu-speaking users to access content in their preferred language with proper RTL support.

**Independent Test**: Can be fully tested by clicking the language toggle and verifying the site switches to Urdu with RTL layout. Delivers immediate value to Urdu-speaking users.

**Acceptance Scenarios**:

1. **Given** a user is viewing the site in English, **When** they click the floating language toggle and select "اردو", **Then** the page reloads with Urdu content and RTL text direction applied to the entire layout.
2. **Given** a user is viewing the site in English, **When** they click the floating language toggle, **Then** they see options for "English" and "اردو" (Urdu in native script).
3. **Given** a user has switched to Urdu, **When** they navigate to another page, **Then** the site remains in Urdu with RTL layout.

---

### User Story 2 - Switch Back to English (Priority: P1)

A user who previously switched to Urdu wants to return to English. They use the floating toggle to switch back, and the site returns to left-to-right (LTR) layout with English content.

**Why this priority**: Users must be able to switch between languages in both directions - this completes the core toggle functionality.

**Independent Test**: Can be fully tested by switching from Urdu to English and verifying LTR layout returns. Essential for complete language switching experience.

**Acceptance Scenarios**:

1. **Given** a user is viewing the site in Urdu (RTL), **When** they click the floating language toggle and select "English", **Then** the page reloads with English content and LTR text direction.
2. **Given** a user switches from Urdu to English, **When** the page loads, **Then** all UI elements return to LTR orientation.

---

### User Story 3 - Persistent Language Preference (Priority: P2)

A user selects Urdu as their preferred language. When they return to the site in a new browser session or navigate across pages, their language preference is remembered.

**Why this priority**: Persistence improves UX by not requiring users to re-select their language on every visit, but the core toggle functionality must work first.

**Independent Test**: Can be tested by selecting a language, closing the browser, returning to the site, and verifying the language preference persists.

**Acceptance Scenarios**:

1. **Given** a user has selected Urdu as their language, **When** they close and reopen the browser and return to the site, **Then** the site displays in Urdu with RTL layout.
2. **Given** a user is navigating through multiple pages, **When** they click internal links, **Then** their language preference is maintained across all page transitions.

---

### User Story 4 - Access Floating Toggle from Any Position (Priority: P2)

A user is scrolling through a long documentation page. They want to switch languages without scrolling back to the top. The floating toggle remains visible and accessible regardless of scroll position.

**Why this priority**: Accessibility of the toggle is important for UX, but secondary to the core language switching functionality.

**Independent Test**: Can be tested by scrolling to the bottom of a long page and verifying the toggle is still visible and functional.

**Acceptance Scenarios**:

1. **Given** a user has scrolled to the bottom of a long page, **When** they look for the language toggle, **Then** the floating button is visible in a fixed position (top-left corner).
2. **Given** a user clicks the floating toggle while scrolled down, **When** the language dropdown appears, **Then** it does not overlap with content and remains fully accessible.

---

### User Story 5 - Graceful Fallback for Missing Translations (Priority: P3)

A content author has not yet translated a specific documentation page to Urdu. When a user visits that page in Urdu mode, the site displays the English content as a fallback rather than showing an error.

**Why this priority**: This is a resilience feature that allows incremental translation - important for maintainability but not core functionality.

**Independent Test**: Can be tested by accessing a page that has no Urdu translation while in Urdu mode and verifying English content displays.

**Acceptance Scenarios**:

1. **Given** a page exists only in English, **When** a user visits that page with Urdu selected, **Then** the English content is displayed as fallback.
2. **Given** the site is in Urdu mode, **When** a user navigates to a page without Urdu translation, **Then** the site remains in RTL layout but shows English content for that page.

---

### User Story 6 - AI-Powered Chapter Translation (Priority: P2)

A logged-in user is reading a chapter in English and wants to read it in Urdu. They click the "Translate to Urdu" button at the top of the chapter, triggering an AI-powered translation that displays the Urdu version of the content.

**Why this priority**: This feature enables content translation at scale, allowing users to contribute to the Urdu translation effort through AI assistance. Requires authentication infrastructure to be in place.

**Independent Test**: Can be tested by logging in, navigating to a chapter, clicking the translate button, and verifying AI-generated Urdu content appears.

**Acceptance Scenarios**:

1. **Given** a logged-in user is viewing a chapter in English, **When** they click the "Translate to Urdu" button at the start of the chapter, **Then** the system displays a loading indicator while AI translation is in progress.
2. **Given** AI translation is complete, **When** the translated content is ready, **Then** the chapter content is replaced with the Urdu translation in RTL layout.
3. **Given** a user is NOT logged in, **When** they view a chapter, **Then** the "Translate to Urdu" button is not visible or is disabled with a "Login to translate" tooltip.
4. **Given** a chapter already has a saved Urdu translation, **When** a logged-in user views it, **Then** the translate button shows "Re-translate" or is hidden.
5. **Given** an AI translation fails (API error, timeout), **When** the error occurs, **Then** the system displays a user-friendly error message and allows retry.

---

### User Story 7 - Save AI Translation (Priority: P2)

After AI translation completes, the logged-in user can optionally save the translation so other users can view the Urdu version without re-translating.

**Why this priority**: Saving translations prevents redundant API calls and allows translations to be reviewed/improved over time.

**Independent Test**: Can be tested by translating a chapter, clicking save, then verifying another user can see the saved translation.

**Acceptance Scenarios**:

1. **Given** an AI translation is displayed, **When** the user clicks "Save Translation", **Then** the Urdu content is persisted and available for all users.
2. **Given** a saved translation exists, **When** any user (logged in or not) visits the Urdu version of the page, **Then** the saved translation is displayed.
3. **Given** a user saves a translation, **When** the save completes, **Then** the system shows a success confirmation.
4. **Given** an AI translation is displayed, **When** the user clicks "Save Translation", **Then** the system requires explicit confirmation before saving to prevent inappropriate content.

---


### Edge Cases

- What happens when a user has JavaScript disabled? (Fallback: language selection should still work via URL-based routing)
- How does the system handle when localStorage is not available? (Fallback: language preference stored in URL path)
- What happens if the user manually changes the URL language prefix? (System respects URL-based language selection)
- How does the toggle behave on very small screens (mobile)? (Toggle remains visible and accessible without blocking content)
- What happens if Urdu fonts fail to load? (System falls back to system fonts that support Arabic script)
- What happens if AI translation API is rate-limited? (Show friendly error, suggest retry later)
- What happens if a user starts translation but navigates away? (Cancel in-progress translation, no partial save)
- What if the same chapter is being translated by multiple users simultaneously? (Last save wins, no conflict resolution needed initially)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a floating language toggle button in a fixed position (top-left corner) that remains visible regardless of scroll position
- **FR-002**: System MUST provide language options for English and Urdu ("English" and "اردو" in native script)
- **FR-003**: System MUST set English as the default language for new visitors
- **FR-004**: System MUST apply RTL (right-to-left) text direction to the entire layout when Urdu is selected
- **FR-005**: System MUST apply LTR (left-to-right) text direction when English is selected
- **FR-006**: System MUST persist the user's language preference across page navigation within the same session
- **FR-007**: System MUST persist the user's language preference across browser sessions (via localStorage or cookies)
- **FR-008**: System MUST fall back to English content when Urdu translation is not available for a specific page
- **FR-009**: System MUST use a globe icon or similar language indicator for the floating toggle button
- **FR-010**: System MUST NOT place language toggle in the navbar - it must be a separate floating component
- **FR-011**: System MUST build successfully even when translation files are empty or missing
- **FR-012**: System MUST support the structure for Urdu translations (placeholder/empty files acceptable initially)
- **FR-013**: Floating toggle MUST NOT block or overlap with main content
- **FR-014**: System MUST use URL-based language routing (e.g., `/ur/docs/...` for Urdu)

#### AI Translation Requirements (Logged-in Users Only)

- **FR-015**: System MUST display a "Translate to Urdu" button at the top of each chapter/doc page for logged-in users only
- **FR-016**: System MUST hide or disable the translate button for non-authenticated users with a "Login to translate" tooltip
- **FR-017**: System MUST show a loading indicator during AI translation process
- **FR-018**: System MUST display the AI-translated content in RTL layout once translation completes
- **FR-019**: System MUST provide a "Save Translation" button after successful AI translation
- **FR-020**: System MUST persist saved translations so they are available to all users viewing the Urdu version
- **FR-021**: System MUST display user-friendly error messages if AI translation fails (timeout, API error, rate limit)
- **FR-022**: System MUST allow retry of failed translations
- **FR-023**: System MUST use an OpenAI-compatible provider (via OPENAI_BASE_URL) for AI translation per constitution
- **FR-024**: System MUST NOT auto-translate without explicit user action (button click)
- **FR-025**: Translate button MUST be positioned at the start of chapter content, not floating

### Key Entities

- **Locale**: Represents a supported language (English: `en`, Urdu: `ur`) with associated text direction (LTR/RTL)
- **Translation Files**: JSON files containing translated strings for UI elements, keyed by locale
- **Content Directory**: Separate directories for localized documentation content per language
- **User Preference**: Stored language selection that determines which locale to display
- **AI Translation**: A machine-generated Urdu translation of chapter content, triggered by logged-in user
- **Saved Translation**: An AI-generated translation that has been persisted for reuse by all users
- **Translation Status**: State of a chapter's translation (untranslated, in-progress, translated, saved)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can switch between English and Urdu within 2 clicks (one to open toggle, one to select language)
- **SC-002**: Language switch completes within 3 seconds on standard broadband connection
- **SC-003**: 100% of existing English documentation remains accessible after i18n implementation
- **SC-004**: Site builds successfully with zero translation files (fallback mode)
- **SC-005**: Floating toggle is visible on 100% of pages without manual action (no scrolling required to find it)
- **SC-006**: Language preference persists correctly in 100% of navigation scenarios within the same session
- **SC-007**: RTL layout is correctly applied to all page elements when Urdu is selected (text, navigation, layout direction)
- **SC-008**: Toggle is accessible and functional on mobile devices (minimum tap target of 44x44 pixels)
- **SC-009**: AI translation completes within 30 seconds for average-length chapters
- **SC-010**: Translate button is visible only to authenticated users on 100% of doc pages
- **SC-011**: Saved translations load faster than re-translating (< 1 second vs 10-30 seconds)
- **SC-012**: Translation errors display actionable messages with retry option

## Assumptions

- The Docusaurus site already has a working base configuration
- No existing navbar language switcher needs to be removed or modified
- Urdu translations will be added incrementally after the infrastructure is in place
- Users have modern browsers that support CSS direction properties and localStorage
- The site uses standard Docusaurus theming that can be extended via swizzling
- Authentication system (Better Auth) is already implemented and functional
- Backend API infrastructure exists for handling translation requests
- An OpenAI-compatible provider is configured via OPENAI_BASE_URL environment variable

## Out of Scope

- Manual human translation workflow (only AI-powered translation)
- Support for additional languages beyond English and Urdu
- Translation review/approval workflow (saved translations are immediately available)
- Translation versioning or rollback
- SEO optimization for Urdu content (can be addressed in future iteration)
- Offline translation capability
- Batch translation of multiple chapters at once

## Clarifications

### Session 2025-12-17

- Q: Should users be required to explicitly confirm before saving AI translations to prevent inappropriate content? → A: Option A - Require explicit user consent before saving translations (e.g., confirm 'Save this translation?') to protect against inappropriate content
