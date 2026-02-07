# Data Model: Curriculum, Modules, and Chapter Overview

## Overview
Since this is a documentation-only feature for an educational textbook, the "data model" represents the conceptual structure and organization of the curriculum content rather than traditional data entities.

## Content Structure Model

### Module Entity
- **id**: String (e.g., "module-1-ros2")
- **title**: String (e.g., "ROS 2 as the robotic nervous system")
- **description**: String (detailed explanation of module purpose)
- **learning_objectives**: Array of strings (specific skills/concepts)
- **prerequisites**: Array of module references (previous modules required)
- **real_world_applications**: Array of strings (industry use cases)
- **integration_points**: Array of capstone components (how it connects to capstone)

### Chapter Entity
- **id**: String (e.g., "module-1-chapter-1")
- **module_id**: String (reference to parent module)
- **title**: String (chapter heading)
- **content_summary**: String (brief overview of content)
- **learning_outcomes**: Array of strings (what student will know/understand)
- **prerequisites**: Array of content references (previous chapters/modules)

### Capstone System Entity
- **id**: String ("capstone-system")
- **title**: String ("Integrated Voice-to-Action Robotics System")
- **description**: String (overview of complete system)
- **components**: Array of component objects (individual system parts)
- **integration_points**: Array of module references (which modules contribute)
- **sim_to_real_features**: Array of strings (capabilities for simulation and real-world)

### Component Entity
- **id**: String (e.g., "voice-interface")
- **name**: String (e.g., "Voice Interface Layer")
- **module_origin**: String (which module provides this component)
- **function**: String (what this component does)
- **interfaces**: Array of interface definitions (how it connects to other components)

## Document Structure Model

### Curriculum Overview Document
- **title**: String ("Curriculum, Modules, and Chapter Overview")
- **word_count**: Integer (between 1500-1800)
- **sections**: Array of section objects (organized content structure)
- **target_audience**: String ("CS/Robotics educators and students")
- **prerequisites**: Array of strings (what reader should know)
- **learning_objectives**: Array of strings (what reader will understand)

### Section Entity
- **id**: String (e.g., "module-philosophy")
- **title**: String (section heading)
- **content_type**: String ("philosophy", "technical", "application", "integration")
- **word_count_target**: Integer (estimated length)
- **dependencies**: Array of other sections this references
- **figures**: Array of figure objects (diagrams, charts, images to include)

### Figure Entity
- **id**: String (e.g., "capstone-architecture-diagram")
- **title**: String (caption for the figure)
- **type**: String ("system-diagram", "flow-chart", "progression-map")
- **description**: String (what the figure shows)
- **placement_section**: String (which section this appears in)
- **purpose**: String (why this figure is needed)

## Relationship Model

### Module Relationships
- **prerequisite_of**: Array of module references (which modules require this one)
- **builds_on**: Array of module references (which modules this depends on)
- **complements**: Array of module references (which modules work together)

### Content Hierarchy
- **parent**: Reference to parent entity (module, section, etc.)
- **children**: Array of child entities (chapters, subsections, etc.)
- **siblings**: Array of peer entities (other chapters in same module)

## Validation Rules
1. Each module must have at least 3 real-world applications defined
2. Each module must clearly define its contribution to the capstone system
3. Prerequisite relationships must form a directed acyclic graph (no circular dependencies)
4. The capstone system must integrate components from all 4 modules
5. Content word count must stay within 1,500-1,800 range
6. All content must be accessible to CS/Robotics students without excessive jargon

## Schema Representation

```yaml
curriculum_overview:
  title: "Curriculum, Modules, and Chapter Overview"
  target_word_count:
    min: 1500
    max: 1800
  modules:
    - id: string
      title: string
      description: string
      learning_objectives: [string]
      prerequisites: [string]
      real_world_applications: [string]
      integration_points: [string]
  sections:
    - id: string
      title: string
      content_type: enum(philosophy, technical, application, integration)
      word_count_target: integer
      dependencies: [string]
  capstone_system:
    id: string
    title: string
    description: string
    components: [component_object]
    integration_points: [string]
    sim_to_real_features: [string]
  figures:
    - id: string
      title: string
      type: enum(system-diagram, flow-chart, progression-map)
      description: string
      placement_section: string
      purpose: string
```
