<!--
Sync Impact Report:
Version change: N/A -> 1.0.0
Added sections: Core Principles (6), Standards and Constraints, Quality & Compliance, Success Criteria
Removed sections: N/A
Modified principles: N/A (new constitution)
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ⚠ pending review
Follow-up TODOs: None
-->

# AI-Native Book with Integrated RAG Chatbot Constitution

## Core Principles

### I. Spec-Driven, Reproducible Content Generation
All book content and project components must be generated from specifications using Spec-Kit Plus workflows. Content generation must be deterministic and reproducible from the repository without manual steps.

### II. Accuracy Verified Against Primary Technical Documentation
All technical claims, code examples, and explanations must be traceable to official documentation, specifications, or source code. No secondary sources or assumptions are acceptable for technical content.

### III. Clarity for Developers and AI Practitioners
Content must be structured and explained in a way that is accessible to both experienced developers and AI practitioners. Technical concepts should be clearly explained with practical examples.

### IV. End-to-End Automation via Claude Code and Spec-Kit Plus
The entire content generation, build, and deployment pipeline must be automated using Claude Code and Spec-Kit Plus. Manual interventions are only acceptable for exceptional circumstances and must be documented.

### V. Strict Context Grounding (NON-NEGOTIABLE)
The RAG chatbot must ground all responses strictly in indexed book content. No hallucinations or external knowledge are acceptable. Selected-text Q&A must restrict context to user-highlighted text only.

### VI. Deterministic Builds and Reproducible Deployments
All code, configurations, and build processes must be reproducible from the repository. Builds must be deterministic, producing identical outputs from identical inputs.

## Standards and Constraints

### Technology Stack Requirements
- Writing & orchestration: Claude Code
- Specs & workflow control: Spec-Kit Plus
- Frontend documentation: Docusaurus
- Hosting: GitHub Pages
- Backend API: FastAPI
- RAG stack:
  - OpenAI Agents / ChatKit SDKs
  - Vector DB: Qdrant Cloud (Free Tier)
  - Metadata & session storage: Neon Serverless Postgres

### Content and Architecture Requirements
- All technical content must be authored using Spec-Kit Plus workflows
- Documentation framework: Docusaurus
- Version control and deployment via GitHub Pages
- No hard-coded knowledge outside retrieved context
- Clear separation between content generation, retrieval, and reasoning

## Quality & Compliance

### Zero Hallucination Tolerance
The chatbot must never generate content that is not directly grounded in the indexed book content. All responses must be verifiable against the book's content.

### Verification and Compliance Requirements
- All code and configurations must be reproducible from repository
- Content generation must be traceable to specifications
- Build processes must be deterministic
- Deployment must be automated via GitHub Pages

## Success Criteria

### Delivery Requirements
- Book successfully published on GitHub Pages
- RAG chatbot embedded and functional within the book
- Chatbot accurately answers book-level and selected-text queries
- Full project reproducible from specs and repository without manual steps

### Quality Standards
- All technical claims verified against official documentation
- Content accessible to target audience (developers and AI practitioners)
- Automated build and deployment pipeline operational
- No hallucinations in chatbot responses

## Governance

This constitution supersedes all other development practices within the project. All project decisions must align with these principles. Amendments to this constitution require explicit documentation of the changes, approval from project maintainers, and a migration plan for existing code and content.

All pull requests and code reviews must verify compliance with these principles. Complexity must be justified with clear benefits to the project's core mission.

**Version**: 1.0.0 | **Ratified**: 2025-06-13 | **Last Amended**: 2025-12-23


