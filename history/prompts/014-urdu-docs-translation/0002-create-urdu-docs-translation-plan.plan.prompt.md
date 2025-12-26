---
id: "0002"
title: "Create Urdu docs translation plan"
stage: "plan"
date: "2025-12-25"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "014-urdu-docs-translation"
branch: "014-urdu-docs-translation"
user: "ary"
command: "sp.plan Translate all docs to Urdu"
labels: ["plan","urdu","translation","docs"]
links:
  spec: "specs/014-urdu-docs-translation/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/014-urdu-docs-translation/plan.md"
tests: []
---

## Prompt

Translate all docs to Urdu

## Response snapshot

Created implementation plan for Urdu Documentation Translation:

**Technical Context:**
- TypeScript 5.6, React 18, Docusaurus 3
- File system storage (docs/ur/)
- No backend changes needed

**Constitution Check:** PASS
- No violations - this is a frontend/docs feature

**Implementation Plan (6 steps):**
1. Create LanguageContext.tsx (state + localStorage)
2. Create LanguageToggle.tsx (EN/UR button)
3. Modify index.tsx (add toggle at end)
4. Modify docs.tsx (add toggle at end)
5. Update docusaurus.config.ts (enable 'ur' locale)
6. Create docs/ur/ folder structure with 17 files

**Translation Rules:**
- Code/commands/URLs: Keep English
- Headings/text: Translate to Urdu
- Key terms: "Urdu (English)" format
- Code blocks: Urdu comments above/below

## Outcome

- ✅ Impact: Implementation plan for Urdu docs with toggle button
- 🧪 Tests: Manual verification only
- 📁 Files: plan.md
- 🔁 Next prompts: /sp.tasks to generate tasks.md
- 🧠 Reflection: Simple feature, no research needed. Constitution check passed easily.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
