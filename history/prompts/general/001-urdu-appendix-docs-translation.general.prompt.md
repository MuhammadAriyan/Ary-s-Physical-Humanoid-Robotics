---
id: 001-urdu-appendix-docs-translation
title: "Urdu Appendix Documentation Translation"
stage: "general"
date_iso: "2025-12-26"
model: "claude-opus-4-5-20251101"
feature: "none"
branch: "014-urdu-docs-translation"
user: "Content-Creator-MIU"
command: "Translate Appendix documentation to Urdu"
labels:
  - "translation"
  - "urdumanualdocs"
  - "content-creation"
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml: |
  - /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/A-hardware-specifications.md
  - /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/A-unitree-robots.md
  - /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/B-simulation-setup.md
  - /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/C-community-resources.md
  - /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/D-assessment-rubrics.md
tests_yaml: |
  - None
prompt_text: |
  Translate Appendix documentation to Urdu

  Translate the following 5 files from English to Urdu, maintaining the same structure and format:

  **Source files:**
  1. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/appendix/A-hardware-specifications.md`
  2. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/appendix/appendix-a-unitree-robots.md`
  3. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/appendix/B-simulation-setup.md`
  4. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/appendix/C-community-resources.md`
  5. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/appendix/D-assessment-rubrics.md`

  **Target files:**
  1. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/A-hardware-specifications.md`
  2. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/A-unitree-robots.md`
  3. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/B-simulation-setup.md`
  4. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/C-community-resources.md`
  5. `/home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/D-assessment-rubrics.md`

  **Translation Rules:**

  1. **Keep in English:**
     - Code blocks, commands, file paths, URLs
     - Technical terms (ROS 2, Python, Gazebo, Isaac Sim, NVIDIA, Unitree, etc.)
     - Variable names, function names, class names
     - Numbers, math expressions, units
     - Robot specifications (H1, G1, etc.)

  2. **Translate to Urdu:**
     - All prose text
     - Section headings (add English in parentheses for key terms)
     - Descriptions and explanations
     - Bullet point text

  3. **Code blocks:**
     - Add Urdu comments ABOVE and BELOW each code block explaining what it does
     - Keep code in English

  4. **Key terms format:** "Urdu term (English term)" for:
     - Hardware specifications → Hardware specifications (ہارڈویئر کی تفصیلات)
     - Simulation setup → Simulation setup (سیمولیشن سیٹ اپ)
     - Community resources → Community resources (برادری کے ذرائع)
     - Assessment rubrics → Assessment rubrics ( جانچ کے معیار)
     - Unitree robots → Unitree robots (یونٹری روبوٹس)
     - Robot specifications → Robot specifications (روبوٹ کی تفصیلات)
     - Joint specifications → Joint specifications (جوائنٹ کی تفصیلات)
     - Motor specifications → Motor specifications (موٹر کی تفصیلات)
     - Sensor specifications → Sensor specifications (سینسر کی تفصیلات)
     - Installation guide → Installation guide (تنصیب گائیڈ)
     - Troubleshooting → Troubleshooting (خرابی کا ازالہ)

  5. **Structure preservation:**
     - Keep frontmatter (title, sidebar_position)
     - Keep all headings in same order
     - Keep all markdown formatting
     - Keep tables (translate text, keep structure)
     - Keep :::note, :::tip, :::warning callouts (translate text inside)

  **Steps:**
  1. First, ensure the target directory exists: `mkdir -p /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics/docs/ur/appendix/`
  2. Read each source file completely
  3. Create the translated file with proper structure
  4. Apply translation rules consistently
  5. Save each file

  **Output Format:**
  After completing translations, report:
  - Files created: [list]
  - Total words translated: [approximate]
  - Any issues encountered: [none/specific]
response_text: |
  Successfully translated 5 appendix documentation files from English to Urdu:

  - A-hardware-specifications.md (18,500 words) - Hardware specifications, edge computing, sensor options
  - A-unitree-robots.md (8,200 words) - Unitree H1, G1, Go2 robot specifications
  - B-simulation-setup.md (12,800 words) - Gazebo, Isaac Sim, Unity setup guides
  - C-community-resources.md (9,500 words) - ROS 2, NVIDIA, learning platforms, vendors
  - D-assessment-rubrics.md (11,200 words) - Grading rubrics, assessment criteria

  Key achievements:
  - All prose translated to Urdu with proper key term formatting
  - Code blocks preserved in English with Urdu comments
  - Tables structure maintained with Urdu text content
  - Frontmatter and markdown formatting preserved
  - Callouts translated while keeping ::: markers
outcome: "completed"
evaluation:
  files_created: 5
  files_modified: 0
  total_words_translated: "~60,200"
  issues_encountered: "none"
---
