# ADR-001: Physical AI Curriculum Architecture for Reference, Capstone, and Conclusion Modules

## Status
Accepted

## Date
2025-12-15

## Context
The Physical AI curriculum requires completion with three final modules: Reference (technical reference materials), Capstone (integrated project), and Conclusion (summary and future outlook). These modules must integrate with the existing curriculum while maintaining educational effectiveness, technical comprehensiveness, and structural consistency.

The decision involves how to structure these final modules in terms of content organization, technical framework integration, and implementation approach. Multiple approaches were considered for organizing the content and integrating various robotics frameworks (ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA systems).

## Decision
We will implement the following architectural decisions for the Reference, Capstone, and Conclusion modules:

1. **Sequential Chapter Numbering System**: Use sequential numbering (04-Reference.md, 05-Capstone.md, 06-Conclusion.md) following Docusaurus conventions to maintain consistency with existing curriculum modules (01-03).

2. **Multi-Framework Technical Reference Architecture**: Create comprehensive technical reference covering ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA systems in a single integrated chapter to provide learners with an understanding of the complete Physical AI ecosystem.

3. **Documentation-Only Implementation Approach**: Focus on documentation creation following Docusaurus standards rather than code implementation to align with curriculum goals of creating educational content and leveraging Docusaurus for effective knowledge delivery.

## Alternatives Considered

### Alternative 1: Alphabetical or Hierarchical Naming
- **Approach**: Use alphabetical naming (Reference.md, Capstone.md, Conclusion.md) or hierarchical naming (reference/index.md, capstone/index.md, conclusion/index.md)
- **Rejected Because**: Alphabetical naming lacks clear sequence, hierarchical naming adds unnecessary complexity for a linear curriculum

### Alternative 2: Single Framework Focus
- **Approach**: Focus on a single framework rather than comprehensive multi-framework reference
- **Rejected Because**: Would limit curriculum comprehensiveness and fail to provide integrated understanding

### Alternative 3: External Linking Only
- **Approach**: Link to external framework documentation rather than creating integrated reference materials
- **Rejected Because**: Would reduce curriculum self-containment and make learning dependent on external resources

### Alternative 4: Combined Documentation and Code Repository
- **Approach**: Maintain both documentation and code examples in a combined repository
- **Rejected Because**: Would complicate the educational focus and fragment the learning experience

## Consequences

### Positive Consequences
- Maintains clear curriculum progression with consistent navigation structure
- Provides comprehensive learning resource that integrates multiple technologies
- Simplifies content delivery and maintains focus on educational objectives
- Enables learners to understand the complete Physical AI ecosystem
- Ensures curriculum self-containment and reduces dependency on external resources

### Negative Consequences
- Requires comprehensive documentation effort to cover all frameworks in depth
- May create longer reference materials that could be overwhelming if not well-organized
- Documentation-only approach means learners must source code examples separately if needed

## References
- Feature specification: `/specs/005-reference-capstone-conclusion/spec.md`
- Implementation plan: `/specs/005-reference-capstone-conclusion/plan.md`
- Research findings: `/specs/005-reference-capstone-conclusion/research.md`
- Curriculum constitution: `/.specify/memory/constitution.md`
