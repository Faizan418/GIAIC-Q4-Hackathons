# Data Model: Reference, Capstone & Conclusion

## Content Entities

### Chapter Entity
- **name**: String (04-Reference, 05-Capstone, or 06-Conclusion)
- **title**: String (display title for the chapter)
- **description**: String (brief description for metadata)
- **sidebar_position**: Integer (position in navigation sidebar)
- **content**: String (main chapter content in Markdown/MDX)
- **frontmatter**: Object (YAML frontmatter with metadata)

### Technical Framework Entity
- **name**: String (ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA)
- **version**: String (recommended version)
- **installation_guide**: String (step-by-step installation instructions)
- **configuration**: Object (configuration parameters and settings)
- **common_commands**: Array<String> (frequently used commands)
- **best_practices**: Array<String> (recommended usage patterns)

### Hardware Configuration Entity
- **component_type**: String (CPU, GPU, sensors, actuators, etc.)
- **minimum_requirements**: Object (minimum specs for functionality)
- **recommended_specifications**: Object (optimal specs for performance)
- **compatibility_notes**: String (notes about compatibility issues)
- **setup_guide**: String (configuration instructions)

### Software Configuration Entity
- **platform**: String (Ubuntu, Windows, etc.)
- **dependencies**: Array<String> (required software packages)
- **environment_variables**: Object (required environment settings)
- **installation_scripts**: Array<String> (automated setup procedures)
- **verification_steps**: Array<String> (steps to verify installation)

## Capstone Project Entities

### Architecture Component Entity
- **name**: String (component name)
- **purpose**: String (function of the component)
- **interfaces**: Array<String> (input/output interfaces)
- **dependencies**: Array<String> (other components it depends on)
- **implementation_guide**: String (how to implement the component)

### VLA Pipeline Entity
- **perception_module**: Object (vision processing components)
- **cognition_module**: Object (language and decision processing)
- **action_module**: Object (motor control and execution)
- **communication_protocol**: String (ROS 2 message types used)
- **integration_points**: Array<String> (where modules connect)

## Relationships

### Chapter → Technical Framework
- One chapter (Reference) contains multiple technical framework references
- Each technical framework has detailed documentation within the Reference chapter

### Technical Framework → Configuration
- Each technical framework has multiple hardware and software configurations
- Configurations provide specific setup guidance for different scenarios

### Capstone Project → Architecture Components
- Capstone project is composed of multiple architecture components
- Components work together to create the complete autonomous humanoid system

### VLA Pipeline → Technical Frameworks
- VLA pipeline integrates multiple technical frameworks
- Each pipeline module may use different underlying technologies

## State Transitions (if applicable)

### Content Review States
- DRAFT → UNDER_REVIEW → NEEDS_REVISION → APPROVED → PUBLISHED
- Each state has specific validation requirements and stakeholders

## Validation Rules

### Content Validation
- Word count must meet specified targets (Reference: ~3,000, Capstone: ~4,000-6,000, Conclusion: ~2,000)
- Technical accuracy must be verified by domain experts
- Links and references must be valid and accessible

### Structure Validation
- Must follow Docusaurus frontmatter requirements
- Heading hierarchy must be consistent
- Sidebar position must maintain proper sequence
