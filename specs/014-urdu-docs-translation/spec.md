# Feature Specification: Urdu Documentation Translation

**Feature Branch**: `014-urdu-docs-translation`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Translate all docs to Urdu"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Language Toggle Access (Priority: P1)

Urdu-speaking users need to access documentation in their native language by clicking a language toggle button on the documentation pages.

**Why this priority**: This is the primary user journey that enables Urdu-speaking users to access content. Without this, the translation feature has no value.

**Independent Test**: Can be tested by visiting docs page, clicking Urdu toggle, and verifying navigation to /ur/docs/

**Acceptance Scenarios**:

1. **Given** user is on docs page, **When** user clicks Urdu toggle button, **Then** page navigates to /ur/docs/ URL path
2. **Given** user is on index page, **When** user clicks Urdu toggle button, **Then** page navigates to /ur/docs/ URL path

---

### User Story 2 - Urdu Content Display (Priority: P1)

Urdu-speaking users can read documentation content in Urdu with proper formatting and key technical terms identified.

**Why this priority**: The core value proposition - users must be able to read translated content.

**Independent Test**: Can be tested by visiting /ur/docs/ and verifying content is in Urdu with English technical terms in parentheses

**Acceptance Scenarios**:

1. **Given** user is on /ur/docs/, **When** user views chapter headings, **Then** headings are in Urdu with English terms in parentheses
2. **Given** user is on /ur/docs/, **When** user reads body text, **Then** text is in Urdu
3. **Given** user is on /ur/docs/, **When** user views code blocks, **Then** code is in English with Urdu comments above and below

---

### User Story 3 - Technical Term Clarity (Priority: P2)

Urdu-speaking users can understand technical terms by seeing both Urdu transliteration and English original.

**Why this priority**: Helps users learn English technical vocabulary while understanding concepts in Urdu.

**Independent Test**: Can be tested by checking that key terms like "Physical AI", "ROS 2", "Gazebo" appear as "فزیکل اے آئی (Physical AI)", etc.

**Acceptance Scenarios**:

1. **Given** user is reading Urdu docs, **When** user encounters technical terms, **Then** terms appear as "Urdu term (English term)" format
2. **Given** user is reading Urdu docs, **When** user sees code blocks, **Then** code syntax remains in English

---

### User Story 4 - Full Documentation Coverage (Priority: P2)

All 17 documentation files are translated to Urdu, covering all course sections and appendices.

**Why this priority**: Users need complete documentation coverage, not just partial.

**Independent Test**: Can be tested by checking that all 17 files exist in docs/ur/ with correct folder structure

**Acceptance Scenarios**:

1. **Given** user navigates to /ur/docs/, **When** user views chapter list, **Then** all 6 parts and appendix sections are available
2. **Given** user clicks any chapter, **When** chapter opens, **Then** content is fully translated to Urdu

---

### Edge Cases

- What happens when user toggles back to English? System should navigate back to /docs/
- How does system handle missing translated files? Show 404 or fallback to English
- What if technical terms have no standard Urdu transliteration? Use phonetic approximation

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide Urdu toggle button on index.tsx page
- **FR-002**: System MUST provide Urdu toggle button on docs.tsx page
- **FR-003**: System MUST navigate to /ur/docs/ when Urdu toggle is clicked
- **FR-004**: System MUST display Urdu content from docs/ur/ folder
- **FR-005**: System MUST preserve English text inside code blocks
- **FR-006**: System MUST add Urdu comments above and below each code block
- **FR-007**: System MUST translate headings, descriptions, and body text to Urdu
- **FR-008**: System MUST format key terms as "Urdu term (English term)"
- **FR-009**: System MUST mirror folder structure from docs/ to docs/ur/
- **FR-010**: System MUST preserve frontmatter with translated title

### Key Entities

- **LanguageToggle**: UI component for switching between English and Urdu
- **TranslatedDoc**: Markdown file in docs/ur/ with Urdu content
- **KeyTermMapping**: Translation of technical terms to Urdu with English in parentheses

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of 17 documentation files are translated to Urdu (all files exist in docs/ur/)
- **SC-002**: 100% of code blocks have Urdu comments added above and below
- **SC-003**: All key technical terms follow "Urdu (English)" format
- **SC-004**: Toggle button appears on both index.tsx and docs.tsx pages
- **SC-005**: Clicking Urdu toggle successfully navigates to /ur/docs/

## Assumptions

- Docusaurus i18n configuration supports 'ur' locale
- No changes needed to RAG indexing for this feature
- No changes needed to Fubuni agent for this feature
- Users will manually translate content using SpecKit + Miu workflow
