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

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Spec-Driven Development (Spec-Kit Plus)
All book content and project components must be generated from specifications using Spec-Kit Plus workflows. Content generation must be deterministic and reproducible from the repository without manual steps.

### II. AI-Native Textbook Approach
The textbook must embrace AI-native methodologies, leveraging AI tools for content creation, validation, and enhancement. All content should reflect modern AI practices in robotics.

### III. Accuracy Verified Against Primary Technical Documentation
All technical claims, code examples, and explanations must be traceable to official documentation, specifications, or source code. No secondary sources or assumptions are acceptable for technical content.

### IV. Clarity for Developers and AI Practitioners
Content must be structured and explained in a way that is accessible to both experienced developers and AI practitioners. Technical concepts should be clearly explained with practical examples.

### V. End-to-End Automation via Claude Code and Spec-Kit Plus
The entire content generation, build, and deployment pipeline must be automated using Claude Code and Spec-Kit Plus. Manual interventions are only acceptable for exceptional circumstances and must be documented.

### VI. Strict Context Grounding (NON-NEGOTIABLE)
The RAG chatbot must ground all responses strictly in indexed book content. No hallucinations or external knowledge are acceptable. Selected-text Q&A must restrict context to user-highlighted text only.

### VII. Deterministic Builds and Reproducible Deployments
All code, configurations, and build processes must be reproducible from the repository. Builds must be deterministic, producing identical outputs from identical inputs.

## Standards and Constraints

### Technology Stack Requirements
- Writing & orchestration: Claude Code
- Specs & workflow control: Spec-Kit Plus
- Frontend documentation: Docusaurus
- Hosting: GitHub Pages
- Robotics frameworks: ROS 2, Gazebo, Unity, NVIDIA Isaac
- AI frameworks: Vision-Language-Action (VLA) models, LLMs

### Content and Architecture Requirements
- All technical content must be authored using Spec-Kit Plus workflows
- Documentation framework: Docusaurus
- Version control and deployment via GitHub Pages
- Content must cover the complete stack from ROS 2 to AI brain (NVIDIA Isaac) to VLA
- Clear separation between simulation and real-world implementation

## Quality & Compliance

### Zero Hallucination Tolerance
All technical content must be accurate and based on official documentation. No speculative or unverified information is acceptable.

### Verification and Compliance Requirements
- All code and configurations must be reproducible from repository
- Content generation must be traceable to specifications
- Build processes must be deterministic
- Deployment must be automated via GitHub Pages
- All robotics concepts must be validated in simulation before real-world application

## Success Criteria

### Delivery Requirements
- Book successfully published on GitHub Pages
- Complete curriculum covering ROS 2, Digital Twin, NVIDIA Isaac, and VLA
- Full project reproducible from specs and repository without manual steps
- Practical examples and labs for each module

### Quality Standards
- All technical claims verified against official documentation
- Content accessible to target audience (developers and AI practitioners)
- Automated build and deployment pipeline operational
- Practical exercises and projects integrated throughout

## Governance

This constitution supersedes all other development practices within the project. All project decisions must align with these principles. Amendments to this constitution require explicit documentation of the changes, approval from project maintainers, and a migration plan for existing code and content.

All pull requests and code reviews must verify compliance with these principles. Complexity must be justified with clear benefits to the project's core mission.

**Version**: 1.0.0 | **Ratified**: 2025-06-13 | **Last Amended**: 2025-12-25


