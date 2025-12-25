---
id: "0001"
title: "Update RAG tool with exact documentation lines"
stage: "misc"
date: "2025-12-25"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "012-docs-landing-animation"
branch: "013-rag-sync-bge-1024"
user: "ary"
command: "update the tool so it also provides the documentation lines exactly so i can verify and people can get to know what they should read next and also modify the chapters"
labels: ["rag","documentation","fubuni-agent"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "backend/rag_retriever.py"
  - "backend/app/agents/fubuni_agent.py"
tests:
  - "Backend health check - passed"
  - "Fubuni agent with RAG - working with exact lines"
---

## Prompt

update the tool so it also provides the documentation lines exactly so i can verify and people can get to know what they should read next and also modify the chapters

## Response snapshot

Modified `backend/rag_retriever.py`:
- Added `detect_chapter_from_content()` for chapter detection
- Added `extract_exact_lines()` to get relevant lines based on query
- Added `format_doc_reference()` with box-formatted output showing exact lines
- Added `search_knowledge_base_structured()` returning (text, chapter, navigate)
- Enhanced `search_knowledge_base()` with formatted references

Modified `backend/app/agents/fubuni_agent.py`:
- Added `CHAPTER_DISPLAY_NAMES` mapping for better UX
- Added `sources` and `exact_lines` fields to `AgentResponse`
- Updated agent instructions to quote exact lines from docs
- Updated `process_message()` to include sources in response

The agent now provides:
- Exact documentation lines with relevance scores
- Chapter and section information
- "READ MORE" links to full docs
- Clear navigation hints telling users what to read next

## Outcome

- ✅ Impact: RAG system now provides exact documentation lines and proper chapter navigation
- 🧪 Tests: Backend healthy, Fubuni agent tested with ROS2 navigation query
- 📁 Files: 2 files modified (rag_retriever.py, fubuni_agent.py)
- 🔁 Next prompts: Push branch to remote
- 🧠 Reflection: Added structured data extraction to RAG for better agent integration

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
