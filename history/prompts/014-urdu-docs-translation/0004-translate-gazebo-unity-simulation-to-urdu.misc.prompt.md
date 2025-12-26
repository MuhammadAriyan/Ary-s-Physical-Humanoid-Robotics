---
id: 0004
title: Translate Gazebo Unity Simulation chapter to Urdu
stage: misc
date: 2025-12-25
surface: agent
model: claude-opus-4-5-20251101
feature: urdu-docs-translation
branch: 014-urdu-docs-translation
user: ary
command: Translate the file at /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/part-3-simulation/03-gazebo-unity-simulation.md to Urdu.
labels: ["documentation","translation","urdu","gazebo","simulation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/part-3-simulation/03-gazebo-unity-simulation.md
  - /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-3-simulation/03-simulation-setup.md
tests: []
---

## Prompt

Translate the file at /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/part-3-simulation/03-gazebo-unity-simulation.md to Urdu.

Translation rules:
1. Translate all headings, descriptions, and body text to Urdu
2. Key technical terms (Gazebo, ROS 2, Physical AI, simulation, etc.) should appear as "Urdu term (English term)" format
3. Keep ALL code blocks, commands, URLs, file paths, and identifiers in English
4. Add Urdu comments ABOVE and BELOW each code block explaining what the code does
5. Preserve all markdown formatting, frontmatter, and structure
6. Preserve all ASCII diagrams and tables
7. Translate table headers and text content to Urdu

Output the complete translated markdown file to: /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-3-simulation/03-simulation-setup.md

## Response snapshot

Successfully translated the entire Gazebo and Unity Simulation chapter to Urdu. The translation includes:
- All headings and body text translated to Urdu with technical terms in "Urdu (English)" format
- All code blocks, commands, URLs, file paths, and identifiers preserved in English
- Urdu comments added above and below each code block explaining functionality
- Markdown formatting, frontmatter, and structure preserved
- Hardware requirements table translated with headers and content in Urdu
- Links and references maintained in original format

Output file created at: /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-3-simulation/03-simulation-setup.md

## Outcome

- ✅ Impact: Added Urdu documentation for simulation chapter enabling Urdu-speaking developers to learn about Gazebo and Unity simulation setup
- 🧪 Tests: No tests added
- 📁 Files: 1 file created (03-simulation-setup.md)
- 🔁 Next prompts: Translate additional chapters to Urdu
- 🧠 Reflection: Translation completed successfully following all specified rules. Technical terms preserved in English with Urdu translations in parentheses. Code blocks retained in original form with explanatory Urdu comments.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Consider adding glossary of technical terms for consistency across translations
