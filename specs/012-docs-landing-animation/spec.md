# Feature Specification: Custom /docs Landing Page with Animated Intro

**Feature Branch**: `012-docs-landing-animation`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Create a custom /docs landing page with animated intro sequence that fades in 'Journey into', then 'Ary's Humanoid Robots and Physical AI', shows a swirling arrow, then reveals the book chapters navigation"

## User Scenarios & Testing

### User Story 1 - Animated Intro Experience (Priority: P1)

As a visitor to the docs page, I want to see an engaging animated intro sequence so that I feel welcomed and the book content feels premium.

**Why this priority**: This is the core experience that differentiates the site. Without it, the page is just a standard docs listing.

**Independent Test**: Can be tested by loading /docs and verifying the animation sequence plays correctly (text fades in, arrow swirls, chapters appear).

**Acceptance Scenarios**:

1. **Given** the user navigates to `/docs`, **When** the page loads, **Then** "Journey into" text should fade in over 0.8 seconds
2. **Given** "Journey into" has faded in, **When** 0.8s passes, **Then** "Ary's Humanoid Robots and Physical AI" text should fade in below it
3. **Given** the title text is visible, **When** 1.6s passes, **Then** a swirling arrow should appear below the text
4. **Given** the arrow is swirling, **When** 2.5s passes, **Then** both text elements should fade out while the arrow decelerates
5. **Given** the intro has faded out, **When** 3.2s passes, **Then** the book chapters navigation should fade in

### User Story 2 - Chapter Navigation (Priority: P1)

As a reader, I want to see all book chapters clearly listed so that I can navigate to the content I want to study.

**Why this priority**: The chapters are the primary navigation mechanism for the book content.

**Independent Test**: Can be tested by waiting for animation to complete and verifying all 6 parts + appendices are visible and clickable.

**Acceptance Scenarios**:

1. **Given** the chapters section has faded in, **When** the user views the page, **Then** all 6 book parts should be visible in the sidebar
2. **Given** the user clicks on a chapter, **When** the navigation is complete, **Then** the user should be taken to that chapter's content
3. **Given** the user is on mobile, **When** the viewport is narrow, **Then** the chapters should display in a responsive layout

### User Story 3 - Dark Mode Support (Priority: P2)

As a user who prefers dark mode, I want the animated intro to respect my theme preference so that my eyes are comfortable.

**Why this priority**: Accessibility and user comfort are important, though the animation works in both modes.

**Independent Test**: Can be tested by toggling system/theme and verifying colors invert correctly.

**Acceptance Scenarios**:

1. **Given** the user's system is in dark mode, **When** the page loads, **Then** all intro text and icons should be white
2. **Given** the user's system is in light mode, **When** the page loads, **Then** all intro text and icons should be black
3. **Given** the user toggles theme mid-animation, **When** the change occurs, **Then** colors should update smoothly

### User Story 4 - Reduced Motion (Priority: P2)

As a user who is sensitive to motion, I want the animation to be reduced or disabled so that I can access content without discomfort.

**Why this priority**: Accessibility requirement for users with vestibular disorders.

**Independent Test**: Can be tested by setting `prefers-reduced-motion: reduce` and verifying animations are minimized.

**Acceptance Scenarios**:

1. **Given** the user has `prefers-reduced-motion: reduce` set, **When** the page loads, **Then** the animation should skip directly to the chapters view
2. **Given** the user has reduced motion enabled, **When** the page is visible, **Then** no swirling or bouncing animations should play

---

### Edge Cases

- What happens if the user navigates away during animation?
- How does the system handle slow network causing delayed script execution?
- What if JavaScript is disabled - does a fallback display appear?
- How does the animation behave on very small screens (320px width)?

## Requirements

### Functional Requirements

- **FR-001**: System MUST display "Journey into" text that fades in over 0.8 seconds
- **FR-002**: System MUST display "Ary's Humanoid Robots and Physical AI" text that fades in with 0.8s delay after first text
- **FR-003**: System MUST display a swirling arrow that appears at 1.6s and follows an unpredictable path
- **FR-004**: System MUST fade out both text elements and decelerate the arrow at 2.5s
- **FR-005**: System MUST display the book chapters navigation at 3.2s with staggered fade-in
- **FR-006**: System MUST support dark mode by inverting colors (black→white)
- **FR-007**: System MUST respect `prefers-reduced-motion` by skipping animations
- **FR-008**: System MUST use existing design system (Inter font, Playfair Display for titles)
- **FR-009**: System MUST use GPU-accelerated animations (transform, opacity only)
- **FR-010**: System MUST display all 6 book parts and 4 appendices in navigation

### Key Entities

- **IntroSequence**: Controls the phased animation timing (intro→title→arrow→exit→chapters)
- **ChapterCard**: Individual chapter navigation item with title, description, and link
- **ArrowAnimation**: Complex SVG animation with swirling path

## Success Criteria

### Measurable Outcomes

- **SC-001**: Animation sequence completes within 4 seconds total
- **SC-002**: 100% of users see the intro animation on first page load
- **SC-003**: All 6 book parts + 4 appendices are accessible via navigation after animation
- **SC-004**: Dark mode users experience correct color inversion
- **SC-005**: Reduced motion users see immediate chapter navigation
- **SC-006**: Page passes Lighthouse accessibility audit (score > 90)

### Non-Functional Requirements

- **Performance**: Animation runs at 60fps on mid-range devices
- **Accessibility**: WCAG 2.1 AA compliant for motion sensitivity
- **Responsive**: Layout adapts to screens from 320px to 4k+ width

## Assumptions

- Existing Docusaurus docs plugin configuration remains unchanged
- Book chapters data comes from existing sidebars.ts configuration
- No theme changes - maintain existing black/white aesthetic
- Use CSS custom properties from existing design system
