---
id: 1
title: Create Physical AI Humanoid Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: san
command: /sp.constitution
labels: [constitution,physical-ai,humanoid-robotics,spec-kit-plus]
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

Create an sp.constitution file for the book "Physical AI & Humanoid Robotics Course" published using Docusaurus, and deployed to GitHub Pages.
The book will include an embedded Retrieval-Augmented Generation (RAG) chatbot built with OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud.
The book's theme is Physical AI & Humanoid Robotics, focusing on embodied intelligence, ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems.
The constitution must define purpose, principles, quality standards, and rules for producing the book using Spec-Kit Plus and Claude Code, and for integrating the RAG chatbot.

Include:

1. Core Principles

Accuracy from authoritative sources

Clarity for CS-level readers

Reproducibility of all code and workflows

Rigor with preference for peer-reviewed and official documentation

2. Standards

APA-style citations

Minimum 50% peer-reviewed sources when citing academic concepts

Zero plagiarism tolerance

Consistent technical terminology

## Response snapshot

Created a comprehensive constitution file for the "Physical AI & Humanoid Robotics Course" book project. The constitution includes six core principles addressing accuracy, clarity, reproducibility, rigor, RAG chatbot excellence, and technical standards compliance. It also defines quality standards with citation requirements and development workflow guidelines. The constitution governs all aspects of the book's development using Spec-Kit Plus methodology, focusing on content accuracy, reproducible examples, and integration of the RAG chatbot system.

## Outcome

- ✅ Impact: Created foundational constitution document that will guide all development of the Physical AI & Humanoid Robotics Course book
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with comprehensive principles and governance
- 🔁 Next prompts: Development can now proceed following the established constitutional principles
- 🧠 Reflection: The constitution provides clear guidelines for content quality, technical standards, and development workflow for the book project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): not applicable