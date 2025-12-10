---
id: 1
title: Update Fubuni Docs Agent constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: ary
command: sp.constitution
labels: ["constitution", "fubuni", "setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
tests:
  - none
---

## Prompt

```text
# Project Constitution – Fubuni Docs Agent

## Vision
Embed a beautiful, always-available AI assistant named **Fubuni** into our existing Docusaurus documentation site. Fubuni will answer any question about our product by using only the documentation and code examples **we explicitly give it**.

## Core Principles
- Agent name is **Fubuni** (always refer to itself by that name)
- Never hallucinate – if it doesn't know, say "I'm not sure yet, teach me!"
- Use **only our own OpenAI-compatible provider** (Groq, Fireworks, OpenRouter, Together, local LLM – configured via OPENAI_BASE_URL)
- Backend written in **Python** using the **official OpenAI Agents SDK** (openai-agents on PyPI)
- Frontend: floating chat bubble in bottom-right → slides out drawer → can be maximized to full-screen and minimized back
- UI **perfectly matches** current Docusaurus theme (colors, fonts, dark/light mode, rounded corners, Infima classes)
- Fully static deployable (no Node server at runtime)
- You (the owner) will later feed it documentation + code examples – the agent must be ready for that ingestion step

## Non-Goals (for now)
- No retrieval / RAG implementation yet (you will add that later)
- No vector DB, no crawling, no embeddings
- Only the agent skeleton + perfect UI

## Tech Stack (locked)
- Python 3.11 + FastAPI + OpenAI Agents SDK
- Docusaurus 3 + React + TypeScript (swizzled components)
- Custom OpenAI provider only
```

## Response snapshot

Updated the project constitution with Fubuni Docs Agent requirements, including principles for agent identity, provider constraints, tech stack compliance, static deployability, non-goals respect, and future extensibility. Also updated the plan template to include constitution compliance checks.

## Outcome

- ✅ Impact: Created new constitution for Fubuni Docs Agent project with 6 core principles
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md and .specify/templates/plan-template.md
- 🔁 Next prompts: Feature specification and planning
- 🧠 Reflection: Constitution now properly defines Fubuni-specific requirements and constraints

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create feature specification based on constitution