---
id: "0002"
title: "i18n Floating Toggle Implementation Plan"
stage: plan
date: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: 010-i18n-floating-toggle
branch: 010-i18n-floating-toggle
user: MuhammadAriyan
command: /sp.plan
labels: ["i18n", "urdu", "rtl", "ai-translation", "docusaurus"]
links:
  spec: specs/010-i18n-floating-toggle/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-i18n-floating-toggle/spec.md (modified)
  - specs/010-i18n-floating-toggle/plan.md (created)
  - specs/010-i18n-floating-toggle/research.md (created)
  - specs/010-i18n-floating-toggle/data-model.md (created)
  - specs/010-i18n-floating-toggle/quickstart.md (created)
  - specs/010-i18n-floating-toggle/contracts/api.yaml (created)
  - CLAUDE.md (modified by agent context update)
tests:
  - N/A (planning phase, no tests created)
---

## Prompt

/sp.plan [Pasted text #1 +39 lines] - avoid using language switcher on navbar because it is hidden

User clarification: Add the if the logged user can translate the content in Urdu in the chapters by pressing a button at the start of each chapter.

Clarification response: AI Translation (auto-translate) - Button triggers AI-powered auto-translation of the chapter content to Urdu in real-time

## Response snapshot

Created comprehensive implementation plan for Docusaurus i18n with floating language toggle and AI-powered chapter translation:

**Key Artifacts Created:**
1. **research.md** - Docusaurus i18n configuration, RTL support, custom toggle approach, AI translation architecture
2. **data-model.md** - ChapterTranslation entity, state transitions, API contracts
3. **plan.md** - Three-phase implementation: i18n infrastructure, AI translation, polish
4. **quickstart.md** - Setup guide with prerequisites and commands
5. **contracts/api.yaml** - OpenAPI spec for translate and translations endpoints

**Key Decisions:**
- Floating toggle positioned top-left (20px, 20px) to avoid collision with chat bubble (bottom-right) and auth button (top-right)
- Native Docusaurus i18n with `localeConfigs` for RTL direction
- AI translation via backend API using OpenAI-compatible provider (constitution compliance)
- Saved translations stored in PostgreSQL (new `chapter_translations` table)
- Swizzle DocItem/Content to inject translate button at top of chapters

**Updated Spec with:**
- User Story 6: AI-Powered Chapter Translation (P2)
- User Story 7: Save AI Translation (P2)
- FR-015 through FR-025: AI translation functional requirements
- SC-009 through SC-012: AI translation success criteria
- Updated assumptions for auth system and OpenAI provider
- Updated out of scope to clarify AI-only translation approach

## Outcome

- Impact: Created complete planning artifacts for i18n feature with AI translation capability
- Tests: N/A (planning phase)
- Files: 6 files created/modified in specs/010-i18n-floating-toggle/
- Next prompts: /sp.tasks to generate implementation tasks
- Reflection: User requirement for AI translation added significant scope but aligns well with existing constitution (OpenAI-compatible providers)

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully
- Graders run and results (PASS/FAIL): Constitution check PASS (all principles verified)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run /sp.tasks to generate actionable implementation tasks
