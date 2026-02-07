---
id: 0001
title: Update-constitution
stage: constitution
date: 2025-12-14
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: sheikh
command: /sp.constitution
labels: ["constitution", "governance", "spec-driven-development"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - ../.specify/memory/constitution.md
tests:
 - none
---

## Prompt

## Project

**AI / Spec-Driven Book: Physical AI & Humanoid Robotics**

## Purpose

This constitution defines the **mandatory rules and structure** for writing and publishing the book using **Docusaurus**, **Spec-Kit Plus**, and **Claude Code**. All content must comply with this document.

## Vision

Teach **Physical AI and Embodied Intelligence** by bridging the digital AI brain with real and simulated humanoid robotic bodies.

## Tooling (Mandatory)

* Docusaurus (TypeScript) for book authoring
* GitHub Pages for deployment
* Spec-Kit Plus for specification governance
* Claude Code for assisted authoring (human validation required)

## Content Structure (Docs Pattern)

All files must follow **Docusaurus docs folder structure**:

```
docs/
├── 01-Introduction.md
├── 02-Setup-Guide.md
├── 03-Overview-Module-and-Chapter.md
├── Module-1-ROS2/
│   ├── 01-Introduction-to-Physical-AI.md
│   ├── 02-Embodied-Intelligence-and-Sensors.md
│   ├── 03-ROS2-Architecture.md
│   ├── 04-Nodes-Topics-Services.md
│   └── 05-ROS2-Packages-and-Launch-Files.md
├── Module-2-Digital-Twin/
│   ├── 06-Gazebo-Setup-and-Simulation.md
│   └── 07-URDF-Physics-and-Unity.md
├── Module-3-AI-Robot-Brain/
│   ├── 08-NVIDIA-Isaac-Platform.md
│   ├── 09-Perception-and-Manipulation.md
│   └── 10-Reinforcement-Learning-and-Sim-to-Real.md
├── Module-4-Vision-Language-Action/
│   ├── 11-Humanoid-Kinematics-and-Locomotion.md
│   ├── 12-Manipulation-and-Human-Robot-Interaction.md
│   └── 13-Conversational-Robotics.md
├── 04-Reference.md
├── 05-Capstone.md
└── 06-Conclusion.md
```

**Note:** The old README.md file is removed; all overview content is included in the above files.

## Quality Principles

* Accuracy through verified sources
* Engineering clarity for CS/Robotics audience
* Reproducible simulations and code
* Physical realism over theory-only claims

## Standards

* All factual claims must be traceable
* Citation style: APA
* Zero plagiarism tolerance

## Validation Criteria

Content is accepted only if:

* Code runs as documented
* Simulations respect real-world physics
* Learning outcomes align with labs and assessments

## Change Control

Any change to scope, tools, or structure requires updating:

* `/sp.constitution`
* `/sp.specify`
* `/sp.plan`

## Authoring Guidelines (Docusaurus)

* Use `.md` or `.mdx` files as per structure
* Keep headings consistent: `#` for module, `##` for chapters, `###` for sections
* Include code blocks, diagrams, and tables using standard Markdown/MDX syntax
* All file names must be sequential and descriptive to maintain order

## Related Governance Documents

All content creation and updates must align with:

* `/sp.specify` – Detailed module, chapter, weekly breakdown, and hardware/software requirements
* `/sp.plan` – Execution workflow, validation, testing, deployment, and change management
---

## Response snapshot

The constitution file has been successfully updated at ../.specify/memory/constitution.md with the following content:

# AI / Spec-Driven Book: Physical AI & Humanoid Robotics Constitution

## Core Principles

### Docusaurus-First Content Structure
All content must follow Docusaurus docs folder structure with sequential numbering. Use .md or .mdx files with consistent heading hierarchy (# for module, ## for chapters, ### for sections).

### Spec-Driven Development Governance
All content creation and updates must align with /sp.specify, /sp.plan, and /sp.constitution. Change control requires updating these governance documents when scope, tools, or structure changes.

### Quality and Verification (NON-NEGOTIABLE)
All factual claims must be traceable through verified sources. Code runs as documented, simulations respect real-world physics, and learning outcomes align with labs and assessments. Zero plagiarism tolerance.

### Tooling Mandate
Mandatory use of Docusaurus (TypeScript) for book authoring, GitHub Pages for deployment, Spec-Kit Plus for specification governance, and Claude Code for assisted authoring with human validation required.

### Academic Standards
Citation style: APA. Engineering clarity for CS/Robotics audience. Physical realism over theory-only claims. Accuracy through verified sources.

### Content Structure Compliance
All files must follow Docusaurus docs folder structure with sequential and descriptive filenames to maintain order.

## Technology and Standards Requirements

Mandatory tooling: Docusaurus (TypeScript), GitHub Pages, Spec-Kit Plus, Claude Code. Standards: APA citation style, reproducible simulations and code, accuracy through verified sources.

## Development Workflow

Use Spec-Kit Plus commands (/sp.specify, /sp.plan, /sp.tasks) for specification governance. Human validation required for Claude Code assistance. Follow Docusaurus authoring guidelines with proper file structure.

## Governance

Constitution supersedes all other practices. Amendments require documentation via /sp.constitution command. All content must comply with this constitution. Changes to scope, tools, or structure require updating /sp.constitution, /sp.specify, and /sp.plan.

**Version**: 1.0.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14

## Outcome

- ✅ Impact: New project constitution created, establishing governance for the Physical AI & Humanoid Robotics book project
- 🧪 Tests: No tests required for constitution creation
- 📁 Files: Updated ../.specify/memory/constitution.md
- 🔁 Next prompts: Project can now proceed with /sp.specify and /sp.plan commands
- 🧠 Reflection: Constitution successfully captures all project requirements and governance structure

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin with /sp.specify to create feature specifications
