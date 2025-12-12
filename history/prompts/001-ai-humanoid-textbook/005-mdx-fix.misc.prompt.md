---
id: 5
title: mdx-fix
stage: misc
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-humanoid-textbook
branch: 001-ai-humanoid-textbook
user: saima
command: null
labels: [mdx, compilation, fix, capstone]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docs/capstone/architecture.md
tests:
  - MDX compilation error resolved
  - Docusaurus build proceeds without syntax errors
  - Less-than symbol properly escaped in documentation
---

## Prompt

Fix MDX compilation error in docs/capstone/architecture.md where `<10ms` was causing JSX parsing issues.

## Response snapshot

Successfully fixed the MDX compilation error at line 134 of docs/capstone/architecture.md by replacing the problematic `<10ms` sequence with `&lt;10ms`. The `<` character was being interpreted as the start of an HTML/JSX tag by the MDX compiler, causing the build to fail. After the fix, the Docusaurus site builds successfully without the syntax error.

## Outcome

- ✅ Impact: Resolved MDX compilation error that was blocking Docusaurus site build
- 🧪 Tests: Docusaurus build proceeds without syntax errors
- 📁 Files: Fixed docs/capstone/architecture.md line 134
- 🔁 Next prompts: Continue with content development and site deployment
- 🧠 Reflection: Proper HTML entity encoding is essential in MDX files to prevent parsing issues

## Evaluation notes (flywheel)

- Failure modes observed: MDX compilation error due to unescaped less-than symbol
- Graders run and results (PASS/FAIL): PASS - build now proceeds successfully
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Complete remaining capstone documentation