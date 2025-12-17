---
id: "0001"
title: "i18n Floating Toggle Specification"
stage: spec
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 010-i18n-floating-toggle
branch: 010-i18n-floating-toggle
user: ary
command: /sp.specify
labels: ["i18n", "docusaurus", "rtl", "urdu", "floating-toggle"]
links:
  spec: specs/010-i18n-floating-toggle/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-i18n-floating-toggle/spec.md
  - specs/010-i18n-floating-toggle/checklists/requirements.md
tests:
  - null
---

## Prompt

Add internationalization (i18n) support to the Docusaurus site with Urdu and English languages, using a floating language toggle button instead of navbar.

Feature Requirements:
1. Users should be able to toggle between English and Urdu languages
2. A floating language switcher button should appear in a fixed position (top-left corner or similar accessible spot)
3. Default language should be English
4. Urdu should display with proper RTL (right-to-left) text direction
5. The language preference should persist across page navigation
6. All existing documentation content should remain accessible in English
7. Urdu translations can be placeholder/empty initially - the structure just needs to support it

UI Requirements:
- Floating button/icon always visible regardless of scroll position
- Use a globe icon or similar language indicator
- On click/hover, show options for "English" and "اردو" (Urdu in native script)
- Compact, non-intrusive design that doesn't block content
- When Urdu is selected, the entire layout should flip to RTL

Technical Constraints:
- Must use Docusaurus built-in i18n system
- No navbar - implement as a custom floating component
- No external translation services needed
- Site should still build and work if translations are missing (fallback to English)
- Component should be injected globally (possibly via swizzling or custom theme component)

## Response snapshot

Created feature specification for Docusaurus i18n with floating language toggle. The spec defines:
- 5 user stories (P1-P3 priority) covering language switching, RTL support, persistence, and fallback behavior
- 14 functional requirements (FR-001 to FR-014) covering all user-specified features
- 8 measurable success criteria (SC-001 to SC-008)
- Edge cases for JavaScript disabled, localStorage unavailable, mobile screens, and font loading
- Clear assumptions and out-of-scope items

Branch created: 010-i18n-floating-toggle
Spec file: specs/010-i18n-floating-toggle/spec.md
Quality checklist: All items pass validation

## Outcome

- Impact: Created complete feature specification ready for planning phase
- Tests: N/A (specification phase)
- Files: 2 files created (spec.md, checklists/requirements.md)
- Next prompts: /sp.clarify or /sp.plan to proceed with implementation planning
- Reflection: Spec covers all user requirements without needing clarification - the user provided comprehensive requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - all checklist items validated
- Prompt variant (if applicable): Standard /sp.specify workflow
- Next experiment (smallest change to try): N/A - workflow completed successfully
