# Data Model: Technical Setup & Lab Architecture Guide

## Content Entities

### Setup Guide Chapter
- **Name**: 02-Setup-Guide.md
- **Type**: Docusaurus Markdown Document
- **Fields**:
  - title: String (required) - "Technical Setup & Lab Architecture Guide"
  - description: String (optional) - Brief overview of the chapter
  - keywords: List<String> (optional) - SEO keywords
  - content: String (required) - Main content body (1,700-1,900 words)
  - diagrams: List<Diagram> (required) - Embedded architecture diagrams
  - tables: List<Table> (required) - Hardware tiers, cost comparisons, responsibility matrices
- **Validation**:
  - Word count: 1,700-1,900 words
  - Must include required diagrams
  - Must include required tables
  - Must follow Docusaurus Markdown format
- **Relationships**: Part of docs/ directory structure

### Diagram Entities

#### Sim Rig → Edge Brain → Robot Architecture Diagram
- **Name**: sim-edge-robot-architecture.svg
- **Type**: Vector Graphic
- **Fields**:
  - title: String (required) - "Simulation to Edge to Robot Architecture"
  - description: String (required) - Explanation of the architecture flow
  - elements: List<DiagramElement> (required) - Components of the architecture
- **Validation**: Must clearly show the flow from simulation environment to edge processing to robot execution

#### Cloud Training → Local Inference Flow Diagram
- **Name**: cloud-training-local-inference-flow.svg
- **Type**: Vector Graphic
- **Fields**:
  - title: String (required) - "Cloud Training to Local Inference Flow"
  - description: String (required) - Explanation of the training/inference process
  - elements: List<DiagramElement> (required) - Components of the process
- **Validation**: Must clearly show the process from cloud-based training to local inference execution

### Table Entities

#### Hardware Tiers Table
- **Name**: hardware-tiers-table
- **Type**: Markdown Table
- **Fields**:
  - title: String (required) - "Hardware Tiers for Different Use Cases"
  - rows: List<TableRow> (required) - Hardware configurations with specifications
  - columns: List<String> (required) - Specifications to compare (CPU, GPU, RAM, etc.)
- **Validation**: Must include at least 3 hardware tiers with clear specifications

#### Cost Comparison Table
- **Name**: cost-comparison-table
- **Type**: Markdown Table
- **Fields**:
  - title: String (required) - "Cost Comparison: On-Prem vs Cloud Solutions"
  - rows: List<TableRow> (required) - Cost factors with values
  - columns: List<String> (required) - Cost categories (Initial, Ongoing, Maintenance, etc.)
- **Validation**: Must include clear cost breakdown for both options

#### Architecture Responsibility Matrix
- **Name**: responsibility-matrix
- **Type**: Markdown Table
- **Fields**:
  - title: String (required) - "Architecture Responsibilities Matrix"
  - rows: List<TableRow> (required) - Responsibilities with ownership
  - columns: List<String> (required) - Responsibility areas (Simulation, Edge AI, Robot, etc.)
- **Validation**: Must clearly delineate responsibilities across different architecture components

## Content Structure

### Sections
1. **Software Stack Overview** - Define Ubuntu + ROS 2 + Gazebo + Isaac requirements
2. **Digital Twin Workstation Requirements** - Detail high-performance computing needs
3. **Jetson Edge AI Kits** - Explain edge processing capabilities and requirements
4. **Sensor Stack Explanation** - Describe camera, IMU, audio sensor requirements
5. **Robot Lab Options** - Compare proxy, humanoid, and premium robot configurations
6. **Cloud vs Local Architecture** - Contrast on-prem vs cloud lab setups

### Content Requirements
- Zero ambiguity in technical specifications
- Clear warnings about compatibility and performance issues
- Operational realism with practical considerations
- Hardware tier tables with cost comparisons
- Architecture responsibility matrices
- Diagrams showing data flows and system relationships
- APA-style citations where appropriate
- Pedagogically sound progression from basic to advanced concepts

## Validation Rules
- All technical claims must be research-backed
- Content must be accessible to students with varying technical backgrounds
- Diagrams must enhance understanding of complex architectural concepts
- Content must be between 1,700-1,900 words
- Must follow Docusaurus documentation standards
- Tables must be clearly formatted with meaningful comparisons
- Warnings and constraints must be prominently highlighted
