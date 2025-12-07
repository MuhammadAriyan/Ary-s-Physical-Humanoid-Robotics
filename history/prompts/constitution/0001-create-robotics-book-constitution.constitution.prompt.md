---
id: 0001
title: Create Robotics Book Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: gemini-pro
feature: none
branch: main
user: user
command: |
  /sp.constitution You are now the Authoring System for my Docusaurus book on **Physical & Humanoid Robotics**.

  Your job is to generate a full, production-quality **/sp.constitution** file for Spec-Kit Plus.

  Create a complete constitution that defines:

  1. Project-wide principles
  2. Writing standards
  3. Formatting rules
  4. Tone, structure, and reading-level conventions
  5. College-notes-inspired chapter structure
  6. Visual design guidelines for a premium Docusaurus book
  7. Best practices for technical clarity, RAG chunkability, and consistency
  8. Personalization tags for future beginner/advanced rewriting
  9. Urdu-translation-friendly guidelines
  10. Code, diagrams, math, glossary, and example standards
  11. Cross-chapter consistency rules
  12. Versioning rules and quality gates
  13. Attribution requirements — include links:
      - GitHub: https://github.com/muhammadariyan
      - Portfolio: https://thebestaryan.vercel.app
      - LinkedIn: https://linkedin.com/in/muhammad-aryan
  14. Any additional standards required to make this book visually appealing, academically strong, and technically sound.

  Make it structured, concise, strict, and written like a real engineering constitution.

  Output ONLY the final constitution.
labels: ["documentation", "constitution", "setup"]
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

/sp.constitution You are now the Authoring System for my Docusaurus book on **Physical & Humanoid Robotics**.

Your job is to generate a full, production-quality **/sp.constitution** file for Spec-Kit Plus.

Create a complete constitution that defines:

1.  Project-wide principles
2.  Writing standards
3.  Formatting rules
4.  Tone, structure, and reading-level conventions
5.  College-notes-inspired chapter structure
6.  Visual design guidelines for a premium Docusaurus book
7.  Best practices for technical clarity, RAG chunkability, and consistency
8.  Personalization tags for future beginner/advanced rewriting
9.  Urdu-translation-friendly guidelines
10. Code, diagrams, math, glossary, and example standards
11. Cross-chapter consistency rules
12. Versioning rules and quality gates
13. Attribution requirements — include links:
    -   GitHub: https://github.com/muhammadariyan
    -   Portfolio: https://thebestaryan.vercel.app
    -   LinkedIn: https://linkedin.com/in/muhammad-aryan
14. Any additional standards required to make this book visually appealing, academically strong, and technically sound.

Make it structured, concise, strict, and written like a real engineering constitution.

Output ONLY the final constitution.

## Response snapshot

Successfully generated and wrote the project constitution to `.specify/memory/constitution.md`. The constitution defines principles for academic rigor, structured content, visual design, RAG optimization, and global accessibility, along with standards for writing, content creation, quality, and governance.

## Outcome

- ✅ Impact: High - Establishes foundational rules for the entire project.
- 🧪 Tests: N/A
- 📁 Files: Created the project's core constitution file.
- 🔁 Next prompts: Proceed with chapter specification and content generation.
- 🧠 Reflection: The process was smooth. The template was successfully populated based on the user's detailed requirements.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A