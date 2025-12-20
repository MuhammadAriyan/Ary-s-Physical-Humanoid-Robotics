---
id: 0002
title: Fubuni Web Search Implementation Plan
stage: plan
date: 2025-12-21
surface: agent
model: claude-opus-4-5-20251101
feature: 010-fubuni-web-search
branch: 010-fubuni-web-search
user: ary
command: /sp.plan
labels: ["web-search", "duckduckgo", "fubuni", "agent-tool", "ui-panel"]
links:
  spec: specs/010-fubuni-web-search/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-fubuni-web-search/plan.md
  - specs/010-fubuni-web-search/research.md
  - specs/010-fubuni-web-search/data-model.md
  - specs/010-fubuni-web-search/quickstart.md
  - specs/010-fubuni-web-search/contracts/chat-api.yaml
tests:
  - null
---

## Prompt

this tool if the data isn't in documentation it's the least priority when it is used it slides from right like the documentation with same cross to cancel thing and in the rare case where doc and chat and websearch sources all 3 sections are there the measurements wil be 40 doc 30 chat 30 source and also add that whenever i say in the input that you have to use rag you are not aallowed to use web seacrh

## Response snapshot

Created comprehensive implementation plan for Fubuni web search integration:

1. **Plan.md**: Full implementation plan with user clarifications:
   - Web search is lowest priority (after RAG)
   - Sources panel slides from right with close button
   - Triple layout: 40% docs, 30% chat, 30% sources
   - "use rag" in message disables web search

2. **Research.md**: Technical research covering:
   - DuckDuckGo library selection (`duckduckgo-search`)
   - Function tool pattern for agent
   - CSS Grid triple layout approach
   - Error handling strategy

3. **Data-model.md**: Entity definitions:
   - WebSearchResult (title, url, snippet)
   - Extended AgentResponse and ChatResponse models
   - State transitions for panel visibility

4. **Contracts/chat-api.yaml**: OpenAPI spec for extended chat endpoint

5. **Quickstart.md**: Step-by-step implementation guide with code examples

## Outcome

- Impact: Planning complete, ready for /sp.tasks
- Tests: None (planning phase)
- Files: 5 planning artifacts created
- Next prompts: /sp.tasks to generate implementation tasks
- Reflection: User clarifications were essential for UI behavior

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - Constitution check passed
- Prompt variant (if applicable): null
- Next experiment: null
