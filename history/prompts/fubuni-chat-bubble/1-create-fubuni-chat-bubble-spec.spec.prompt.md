---
id: 1
title: Create Fubuni chat bubble spec
stage: spec
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: fubuni-chat-bubble
branch: 007-fubuni-chat-bubble
user: ary
command: sp.specify
labels: ["feature", "spec", "fubuni", "chat"]
links:
  spec: specs/007-fubuni-chat-bubble/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-fubuni-chat-bubble/spec.md
  - specs/007-fubuni-chat-bubble/checklists/requirements.md
tests:
  - none
---

## Prompt

```text
As a visitor of our Docusaurus docs,
I want a floating bubble with the Fubuni icon in the bottom-right corner
so that I can click it and talk to Fubuni instantly.

Acceptance Criteria
1. Bubble is 60×60 px, rounded, with Fubuni logo/icon, subtle pulse animation
2. Click → smooth slide-out drawer from the right (420px wide)
3. Drawer has header "Chat with Fubuni" + your logo
4. Expand button (↗) in header → opens full-screen modal (90% width/height)
5. In full-screen mode the button becomes minimize (↙)
6. Entire UI inherits Docusaurus theme automatically (dark/light mode, primary color, fonts)
7. Messages from Fubuni start with "Fubuni:" or have a cute avatar
8. Streaming responses (token-by-token)
9. Backend is a working FastAPI + OpenAI Agents SDK agent that:
   - Uses your custom provider (OPENAI_BASE_URL + API key)
   - Identifies itself as Fubuni
   - Is ready to receive future tools/documentation
10. Works locally and in static Docusaurus build
```

## Response snapshot

Created comprehensive feature specification for the Fubuni chat bubble feature, including user stories for instant access, full-screen experience, and theme consistency. Defined 14 functional requirements covering UI, backend, and integration aspects. Created success criteria with measurable outcomes and identified key entities and edge cases.

## Outcome

- ✅ Impact: Complete spec ready for planning phase with 3 prioritized user stories
- 🧪 Tests: none
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Planning and task generation
- 🧠 Reflection: Spec captures all acceptance criteria with clear requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create implementation plan