---
id: "009"
title: "Translate Part 5 Humanoid documentation to Urdu"
stage: "general"
date: "2025-12-26"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "none"
branch: "014-urdu-docs-translation"
user: "Content-Creator-MIU"
command: "Translate Part 5 Humanoid documentation to Urdu"
labels: ["urdu","translation","documentation","humanoid-robotics"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-5-humanoid/05a-week-11-12-overview.md
  - /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-5-humanoid/05-humanoid-development.md
tests:
  - None
---

## Prompt

Translate Part 5 Humanoid documentation to Urdu

Translate the following 2 files from English to Urdu, maintaining the same structure and format:

**Source files:**
1. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/part-5-humanoid/05a-week-11-12-overview.md`
2. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/part-5-humanoid/05-humanoid-robot-development.md`

**Target files:**
1. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-5-humanoid/05a-week-11-12-overview.md`
2. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-5-humanoid/05-humanoid-development.md`

## Translation Rules:

1. **Keep in English:**
   - Code blocks, commands, file paths, URLs
   - Technical terms (ROS 2, Python, C++, API, etc.)
   - Variable names, function names, class names
   - Numbers, math expressions, units

2. **Translate to Urdu:**
   - All prose text
   - Section headings (add English in parentheses for key terms)
   - Descriptions and explanations
   - Bullet point text

3. **Code blocks:**
   - Add Urdu comments ABOVE and BELOW each code block explaining what it does
   - Keep code in English

4. **Key terms format:** "Urdu term (English term)" for:
   - Humanoid robot → Humanoid robot (انسان نما روبوٹ)
   - Kinematics → Kinematics (کینیمیٹکس)
   - Locomotion → Locomotion (حرکت)
   - ROS 2 → ROS 2
   - Forward kinematics → Forward kinematics (آگے کی کینیمیٹکس)
   - Inverse kinematics → Inverse kinematics (الٹی کینیمیٹکس)
   - Bipedal → Bipedal (دو پاؤں والا)
   - Manipulation → Manipulation (ہیرا پھیری)
   - Dynamics → Dynamics (حرکیات)
   - Trajectory → Trajectory (ٹریکٹری)

5. **Structure preservation:**
   - Keep frontmatter (title, sidebar_position)
   - Keep all headings in same order
   - Keep all markdown formatting
   - Keep :::note and :::tip callouts (translate text inside)

## Response snapshot

- Files created: /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-5-humanoid/05a-week-11-12-overview.md, /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-5-humanoid/05-humanoid-development.md
- Total words translated: ~12,000 words (5a-week-11-12-overview: ~4,000 words, 05-humanoid-development: ~8,000 words)
- Key technical terms translated with parenthetical English: Humanoid robot (انسان نما روبوٹ), Kinematics (کینیمیٹکس), Locomotion (حرکت), Forward kinematics (آگے کی کینیمیٹکس), Inverse kinematics (الٹی کینیمیٹکس), Bipedal (دو پاؤں والا), Manipulation (ہیرا پھیری), etc.
- Code blocks preserved in English with Urdu comments added above and below
- All markdown structure preserved including frontmatter, tables, and callouts

## Outcome

- ✅ Impact: Completed Urdu translation of Part 5 Humanoid documentation including Week 11-12 overview (546 lines) and Chapter 5 Humanoid Robot Development (2852 lines)
- 🧪 Tests: None
- 📁 Files: 2 translated files created in /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/part-5-humanoid/
- 🔁 Next prompts: Continue with Part 6 Conversational Robotics Urdu translation
- 🧠 Reflection: The translation followed established patterns from previous Urdu translation work. The 05-humanoid-development.md file already existed with partial translation from a previous session, so it was updated with additional content. All technical terms maintained English with Urdu parentheticals as per style guide.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
