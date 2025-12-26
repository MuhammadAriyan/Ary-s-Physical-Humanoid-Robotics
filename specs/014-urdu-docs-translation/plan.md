# Implementation Plan: Urdu Documentation Translation

**Branch**: `014-urdu-docs-translation` | **Date**: 2025-12-25 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/014-urdu-docs-translation/spec.md`

## Summary

Add Urdu language support to the Physical Humanoid Robotics documentation site:
1. Add language toggle button on index.tsx and docs.tsx pages
2. Create 17 translated Urdu markdown files in docs/ur/
3. Configure Docusaurus for Urdu locale ('ur')
4. Use "Urdu term (English term)" format for key terms
5. Keep code in English with Urdu comments above/below code blocks

## Technical Context

**Language/Version**: TypeScript 5.6, React 18, Docusaurus 3
**Primary Dependencies**: React, TypeScript, Docusaurus i18n
**Storage**: File system (markdown files in docs/ur/)
**Testing**: Manual verification
**Target Platform**: Web (Docusaurus static site)
**Project Type**: Documentation site (static)
**Performance Goals**: No impact (static content)
**Constraints**: Keep code/commands/URLs in English; Only translate text content
**Scale/Scope**: 17 markdown files, 6 chapters + appendix

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Verify compliance with Fubuni Docs Agent Constitution:
- Agent MUST be named "Fubuni" and never hallucinate - N/A (this feature is docs UI)
- Implementation MUST use only OpenAI-compatible providers - N/A (no backend changes)
- Backend MUST be in Python 3.11 using official OpenAI Agents SDK - N/A (no backend changes)
- Frontend MUST perfectly match Docusaurus theme - YES (will match existing theme)
- Implementation MUST be fully static deployable - YES (markdown files only)
- Initial implementation MUST NOT include vector DB - N/A (no RAG changes)
- System architecture MUST support later documentation ingestion - YES (Docusaurus i18n)

**Result**: PASS - No constitution violations. This is a frontend/documentation feature.

## Project Structure

### Documentation (this feature)

```text
specs/014-urdu-docs-translation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # N/A - no research needed
├── data-model.md        # N/A - no data model needed
├── quickstart.md        # N/A - no quickstart needed
├── contracts/           # N/A - no API contracts needed
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── components/
│   └── Language/        # NEW - Language context and toggle
│       ├── LanguageContext.tsx
│       └── LanguageToggle.tsx
├── pages/
│   ├── index.tsx        # MODIFY - Add toggle at end
│   └── docs.tsx         # MODIFY - Add toggle at end

docs/
├── part-1-foundations/  # Existing English docs
├── part-2-ros2/
├── part-3-simulation/
├── part-4-isaac/
├── part-5-humanoid/
├── part-6-conversational/
├── appendix/
└── ur/                  # NEW - Urdu docs (17 files)
    ├── part-1-foundations/
    ├── part-2-ros2/
    ├── part-3-simulation/
    ├── part-4-isaac/
    ├── part-5-humanoid/
    ├── part-6-conversational/
    └── appendix/

docusaurus.config.ts     # MODIFY - Enable Urdu locale
```

**Structure Decision**: Adding Language components folder for language toggle functionality. Creating docs/ur/ mirror of docs/ folder structure for Urdu content.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |

## Phase 0: Research (if needed)

**Research required**: NO - Translation workflow is clearly defined in spec.

**Key decisions from spec**:
- Toggle navigates to `/ur/docs/` (Docusaurus locale-based routing)
- Key terms use format "Urdu term (English term)"
- Code blocks keep English, add Urdu comments above/below

## Phase 1: Design & Contracts

**Data model**: N/A - No data structures needed for static documentation

**API contracts**: N/A - No API changes (static content only)

**Implementation Plan**:

### 1. Create LanguageContext.tsx
- Language state management with localStorage
- `language` state ('en' | 'ur')
- `setLanguage()` function
- localStorage persistence key: 'urduMode'

### 2. Create LanguageToggle.tsx
- Button component with EN/UR toggle
- Clicking toggles language and navigates to appropriate docs URL
- Style matching existing theme

### 3. Modify index.tsx
- Import LanguageToggle
- Add toggle at end of page (after chapter cards)
- Use LanguageContext to control display

### 4. Modify docs.tsx
- Import LanguageToggle
- Add toggle at end of page
- Use LanguageContext to control display

### 5. Update docusaurus.config.ts
- Add 'ur' to i18n.locales
- Configure Urdu locale direction: RTL
- Set defaultLocale to 'en'

### 6. Create docs/ur/ folder structure
- Mirror docs/ folder structure
- Create 17 translated markdown files
- Apply translation rules from spec

## Phase 2: Tasks (generated by /sp.tasks)

Run `/sp.tasks` to generate executable tasks.md with testable steps.
