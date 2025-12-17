# Feature Specification: Custom Header/Navbar Component

**Feature Branch**: `009-custom-header`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Create a custom header/navbar component for the Docusaurus site with logo, multi-language dropdown, GitHub/LinkedIn links, and Better Auth integration"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - View Site Branding and Navigation (Priority: P1)

As a visitor, I want to see the site name/logo prominently displayed in the header so I can identify the website and navigate to the homepage by clicking it.

**Why this priority**: The logo/branding is the foundational element of any header - without it, users cannot identify the site or navigate home. This is the minimum viable header.

**Independent Test**: Can be fully tested by loading any page and verifying the logo displays correctly, clicking it navigates to homepage, and it uses the correct Playfair Display font.

**Acceptance Scenarios**:

1. **Given** I am on any page of the site, **When** I view the header, **Then** I see "Ary's Physical & Humanoid Robotics" displayed on the left side in Playfair Display font
2. **Given** I am on a documentation page, **When** I click the site logo/name, **Then** I am navigated to the homepage
3. **Given** I am viewing on mobile (< 768px), **When** I view the header, **Then** the logo is still visible and clickable

---

### User Story 2 - Authenticate via Header (Priority: P1)

As a user, I want to sign in or sign out directly from the header so I can access authenticated features without navigating away from my current page.

**Why this priority**: Authentication is critical for the chat feature (Fubuni) which requires login. Users need immediate access to auth controls from any page.

**Independent Test**: Can be fully tested by clicking Sign In button, completing authentication, verifying user state displays, and signing out.

**Acceptance Scenarios**:

1. **Given** I am not logged in, **When** I view the header, **Then** I see a "Sign In" button on the right side
2. **Given** I am not logged in, **When** I click "Sign In", **Then** the AuthModal opens allowing me to sign in or sign up
3. **Given** I am logged in, **When** I view the header, **Then** I see my avatar (or initial) and can access a dropdown menu
4. **Given** I am logged in, **When** I click my avatar and select "Sign Out", **Then** I am logged out and the "Sign In" button reappears
5. **Given** I am logged in, **When** I view the dropdown, **Then** I see my name/email displayed

---

### User Story 3 - Access External Profiles (Priority: P2)

As a visitor, I want to access the author's GitHub and LinkedIn profiles directly from the header so I can learn more about them or connect professionally.

**Why this priority**: External links provide credibility and connection opportunities but are not essential for core site functionality.

**Independent Test**: Can be fully tested by clicking each icon and verifying correct external URLs open in new tabs.

**Acceptance Scenarios**:

1. **Given** I am on any page, **When** I view the header, **Then** I see GitHub and LinkedIn icons
2. **Given** I am on any page, **When** I click the GitHub icon, **Then** https://github.com/MuhammadAriyan opens in a new tab
3. **Given** I am on any page, **When** I click the LinkedIn icon, **Then** https://www.linkedin.com/in/muhammad-aryan opens in a new tab
4. **Given** I hover over either icon, **When** I wait briefly, **Then** I see a tooltip indicating the destination

---

### User Story 4 - Select Display Language (Priority: P3)

As an international visitor, I want to select my preferred language from a dropdown so I can view content in my native language when translations are available.

**Why this priority**: i18n support is future-proofing - currently only English is available. The structure should be in place for easy expansion.

**Independent Test**: Can be fully tested by clicking language dropdown, selecting a language, and verifying the selection is reflected (initially English-only, structure ready for expansion).

**Acceptance Scenarios**:

1. **Given** I am on any page, **When** I view the header, **Then** I see a language selector dropdown showing the current language (English)
2. **Given** the dropdown is closed, **When** I click the language selector, **Then** a dropdown opens showing available languages
3. **Given** additional languages are configured, **When** I select a different language, **Then** the site content switches to that language
4. **Given** I am on mobile, **When** I access the language selector, **Then** it is accessible within the mobile menu

---

### User Story 5 - Navigate on Mobile Devices (Priority: P2)

As a mobile user, I want to access all header features through a hamburger menu so I can navigate the site effectively on smaller screens.

**Why this priority**: Mobile responsiveness is essential for accessibility - a significant portion of users access sites on mobile devices.

**Independent Test**: Can be fully tested by viewing on a mobile viewport, opening hamburger menu, and verifying all navigation items are accessible.

**Acceptance Scenarios**:

1. **Given** I am viewing on mobile (< 768px), **When** I view the header, **Then** I see a hamburger menu icon instead of the full navigation
2. **Given** I am on mobile, **When** I tap the hamburger icon, **Then** a mobile menu opens with all navigation options
3. **Given** the mobile menu is open, **When** I tap outside the menu or tap close, **Then** the menu closes
4. **Given** the mobile menu is open, **When** I view the menu, **Then** I see language selector, external links, and auth controls

---

### Edge Cases

- What happens when authentication fails? The AuthModal should display appropriate error messages without affecting header state
- How does the header behave during auth loading state? Show a loading indicator on the auth button
- What happens if external links fail to load? Links should still be clickable, browser handles the error
- How does the header appear during page transitions? Header remains fixed and stable during navigation
- What happens when the user has a very long name? Truncate with ellipsis after reasonable length (20 characters)
- How does dark mode affect the header? All elements should adapt to dark theme using existing CSS variables

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display site logo/name "Ary's Physical & Humanoid Robotics" in Playfair Display font, left-aligned
- **FR-002**: System MUST navigate to homepage when logo/name is clicked
- **FR-003**: System MUST display a language selector dropdown with current language indicator
- **FR-004**: System MUST support i18n structure for adding future language options (currently English only)
- **FR-005**: System MUST display GitHub icon linking to https://github.com/MuhammadAriyan (opens in new tab)
- **FR-006**: System MUST display LinkedIn icon linking to https://www.linkedin.com/in/muhammad-aryan (opens in new tab)
- **FR-007**: System MUST display "Sign In" button when user is not authenticated
- **FR-008**: System MUST open AuthModal when "Sign In" button is clicked
- **FR-009**: System MUST display user avatar/initial when user is authenticated
- **FR-010**: System MUST provide dropdown menu with user info and "Sign Out" option when authenticated
- **FR-011**: System MUST integrate with existing Better Auth hooks (useAuth, AuthModal)
- **FR-012**: System MUST use glassmorphism styling matching existing luxury theme
- **FR-013**: System MUST be fixed position at top of viewport with appropriate z-index
- **FR-014**: System MUST be 80px height on desktop, 70px on mobile
- **FR-015**: System MUST collapse into hamburger menu on viewports < 768px
- **FR-016**: System MUST support dark mode via [data-theme='dark'] selector
- **FR-017**: System MUST include smooth hover animations on interactive elements
- **FR-018**: System MUST be keyboard navigable with visible focus states
- **FR-019**: System MUST include appropriate ARIA labels for accessibility

### Key Entities

- **Header**: The main navigation component containing all sub-elements (logo, nav items, auth controls)
- **LanguageSelector**: Dropdown component for language selection, stores current locale preference
- **AuthState**: User authentication state (logged in/out, user info) from existing Better Auth context
- **NavItem**: Individual navigation element (external link, dropdown trigger)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can identify the site and navigate to homepage within 1 second of page load
- **SC-002**: Users can access sign in/sign out functionality within 2 clicks from any page
- **SC-003**: Users can access external profile links (GitHub, LinkedIn) within 1 click
- **SC-004**: Mobile users can access all header functionality through hamburger menu within 2 taps
- **SC-005**: Header renders correctly and remains functional across all supported viewports (320px - 2560px)
- **SC-006**: All interactive elements have visible focus states for keyboard navigation
- **SC-007**: Header maintains consistent appearance during page navigation (no flicker/layout shift)
- **SC-008**: Dark mode toggle reflects correctly in header within 0.3 seconds
- **SC-009**: 100% of header interactive elements pass accessibility audit (ARIA labels, keyboard nav, color contrast)

## Assumptions

- The existing Better Auth integration (useAuth hook, AuthModal component) is functional and will be reused
- The existing glassmorphism CSS variables and luxury theme are available and should be used
- Docusaurus theme customization via src/theme/ is the appropriate integration point
- Only English language is currently needed, but structure should support future i18n expansion
- SVG icons will be used for GitHub and LinkedIn (inline or component-based)
- The user's avatar will fall back to initials if no image is available

## Out of Scope

- Implementing actual multi-language content translation (only the selector UI)
- Search functionality in the header
- Notification badges or alerts
- Breadcrumb navigation
- Secondary navigation menus
- Cookie consent banners
