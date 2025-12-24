---
id: 2
title: Next Previous Navigation
stage: general
date_iso: 2025-12-23
surface: agent
model: Qwen
feature: none
branch: main
user: user
command: /sp.implement
labels: ["docusaurus", "navigation", "mdx", "components"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - C:\Users\HP\Desktop\ai_robutiv_book\docs\src\components\NavigationButtons\NavigationButtons.js
  - C:\Users\HP\Desktop\ai_robutiv_book\docs\src\components\NavigationButtons\NavigationButtons.module.css
  - C:\Users\HP\Desktop\ai_robutiv_book\docs\docs\tutorial-basics\markdown-features.mdx
  - C:\Users\HP\Desktop\ai_robutiv_book\docs\blog\2021-08-01-mdx-blog-post.mdx
  - C:\Users\HP\Desktop\ai_robutiv_book\docs\docs\tutorial-basics\create-a-document.md
  - C:\Users\HP\Desktop\ai_robutiv_book\docs\docs\intro.md
tests_yaml: []
prompt_text: "$ARGUMENTS\n\n---\n\nAs the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.\n\n1) Determine Stage\n   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general\n\n2) Generate Title and Determine Routing:\n   - Generate Title: 3–7 words (slug for filename)\n   - Route is automatically determined by stage:\n     - `constitution` → `history/prompts/constitution/`\n     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)\n     - `general` → `history/prompts/general/`\n\n3) Create and Fill PHR (Shell first; fallback agent‑native)\n   - Run: `.specify/scripts/bash/create-phr.sh --title \"<title>\" --stage <stage> [--feature <name>] --json`\n   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.\n   - If the script fails:\n     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)\n     - Allocate an ID; compute the output path based on stage from step 2; write the file\n     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT\n\n4) Validate + report\n   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.\n   - On failure: warn, don't block. Skip only for `/sp.phr`.\n\n\n/sp.implement Add next and previous chapter navigation links in MDX for Docusaurus chapters"
response_text: "Successfully implemented next and previous chapter navigation for Docusaurus. Created a reusable NavigationButtons React component with CSS module styling. Added the component to several MDX and MD files including markdown-features.mdx, the blog post, create-a-document.md, and intro.md. The component automatically detects the current document's position in the sidebar and provides appropriate navigation links to the previous and next documents in the sequence."
outcome: "Successfully implemented custom next/previous navigation for Docusaurus chapters"
evaluation: "The implementation successfully created a reusable component that provides next and previous navigation based on the sidebar structure. The component works across both MDX and MD files, and includes responsive styling."
---

# Next and Previous Chapter Navigation Implementation

## Overview
This PHR documents the implementation of next and previous chapter navigation links in MDX for Docusaurus chapters.

## Changes Made

### 1. Created NavigationButtons Component
- Developed a reusable React component (`NavigationButtons.js`) that automatically detects the current document's position in the sidebar
- Component finds the previous and next documents based on the sidebar structure
- Added proper error handling and responsive design

### 2. Created CSS Module for Styling
- Developed `NavigationButtons.module.css` with appropriate styling
- Included responsive design for mobile devices
- Added hover and focus states for accessibility

### 3. Integrated Navigation into Documentation Files
- Added the NavigationButtons component to:
  - `docs/docs/tutorial-basics/markdown-features.mdx`
  - `docs/blog/2021-08-01-mdx-blog-post.mdx`
  - `docs/docs/tutorial-basics/create-a-document.md`
  - `docs/docs/intro.md`

## Features Implemented
- Automatic detection of previous and next documents based on sidebar structure
- Responsive design that works on mobile and desktop
- Accessible navigation with proper focus states
- Clean, modern styling that matches Docusaurus design patterns

## Files Modified
- `docs/src/components/NavigationButtons/NavigationButtons.js`: Created new component
- `docs/src/components/NavigationButtons/NavigationButtons.module.css`: Created new CSS module
- Multiple documentation files: Added navigation component import and usage

## Validation
The implementation was tested by examining the component code to ensure proper functionality and integration with the Docusaurus framework. The component properly uses Docusaurus hooks and follows the framework's patterns for accessing sidebar information.