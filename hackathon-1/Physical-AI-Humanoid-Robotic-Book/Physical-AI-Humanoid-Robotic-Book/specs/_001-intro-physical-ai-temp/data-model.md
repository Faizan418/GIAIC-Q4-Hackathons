# Data Model: Introduction to Physical AI & Humanoid Robotics

## Content Entities

### Introduction Chapter
- **Name**: 01-Introduction.md
- **Type**: Docusaurus Markdown Document
- **Fields**:
  - title: String (required) - "Introduction to Physical AI & Humanoid Robotics"
  - description: String (optional) - Brief overview of the chapter
  - keywords: List<String> (optional) - SEO keywords
  - content: String (required) - Main content body (1,500-1,700 words)
  - diagrams: List<Diagram> (required) - Embedded diagrams
- **Validation**:
  - Word count: 1,500-1,700 words
  - Must include required diagrams
  - Must follow Docusaurus Markdown format
- **Relationships**: Part of docs/ directory structure

### Diagram Entities

#### Digital AI → Physical AI Pipeline Diagram
- **Name**: digital-ai-to-physical-ai-pipeline.svg
- **Type**: Vector Graphic
- **Fields**:
  - title: String (required) - Descriptive title
  - description: String (required) - Explanation of the diagram
  - elements: List<DiagramElement> (required) - Components of the diagram
- **Validation**: Must clearly show transition from digital to physical AI

#### Sense → Think → Act Loop Diagram
- **Name**: sense-think-act-loop.svg
- **Type**: Vector Graphic
- **Fields**:
  - title: String (required) - Descriptive title
  - description: String (required) - Explanation of the diagram
  - elements: List<DiagramElement> (required) - Components of the loop
- **Validation**: Must clearly show the cyclical process of perception, decision, and action

## Content Structure

### Sections
1. **What is Physical AI?** - Define core concept with examples
2. **Embodied Cognition Explained** - Explain the theory behind embodied intelligence
3. **Limitations of Disembodied AI** - Contrast with traditional digital AI
4. **Humanoids in Human-Centered Worlds** - Explain humanoid form factor advantages
5. **AI-Native Textbook Philosophy** - Introduce simulation-first learning approach

### Content Requirements
- Conceptual clarity over technical implementation
- Research-backed explanations
- No heavy code examples
- APA-style citations where appropriate
- Pedagogically sound progression from basic to advanced concepts

## Validation Rules
- All claims must be research-backed
- Content must be accessible to students with varying technical backgrounds
- Diagrams must enhance understanding of abstract concepts
- Content must be between 1,500-1,700 words
- Must follow Docusaurus documentation standards
