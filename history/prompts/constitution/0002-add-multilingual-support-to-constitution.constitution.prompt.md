---
id: 0002
title: Add Multilingual Support to Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: gemini-pro
feature: none
branch: constitution
user: user
command: /sp.constitution add that it also should have language option we can switch between eng urdu and other languages
labels: ["documentation", "constitution", "i18n", "enhancement"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution add that it also should have language option we can switch between eng urdu and other languages

## Response snapshot

Successfully amended the project constitution to include a new core principle requiring multilingual support. The version was bumped to 1.1.0, and Principle VI was updated to mandate a language switcher on the Docusaurus site for English, Urdu, and other future languages.

## Outcome

- ✅ Impact: High - This change introduces a core functional requirement for the final product and affects future development and content strategy.
- 🧪 Tests: N/A
- 📁 Files: Updated the project's core constitution file at `.specify/memory/constitution.md`.
- 🔁 Next prompts: Implement the i18n feature in Docusaurus or proceed with chapter content creation.
- 🧠 Reflection: The amendment was straightforward. Identifying the correct principle to modify (Principle VI) was more effective than adding a new, separate one. This keeps the constitution concise.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A