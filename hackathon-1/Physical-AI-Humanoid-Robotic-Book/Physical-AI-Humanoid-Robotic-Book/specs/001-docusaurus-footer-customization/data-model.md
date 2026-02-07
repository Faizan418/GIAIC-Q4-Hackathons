# Data Model: Docusaurus Footer Customization

## Entity: Footer Link Groups
- **Name**: FooterLinkGroups
- **Description**: Collections of related navigation links (Docs, Modules, More)
- **Fields**:
  - name: string (e.g., "Docs", "Modules", "More")
  - links: array of FooterLink objects
- **Relationships**: Contains multiple FooterLink entities

## Entity: Footer Link
- **Name**: FooterLink
- **Description**: Navigation elements that point to specific documentation pages or external resources
- **Fields**:
  - label: string (display text for the link)
  - to: string (internal document path) OR href: string (external URL)
- **Validation**: Must have either 'to' or 'href' property, but not both

## Entity: Documentation Link
- **Name**: DocumentationLink
- **Description**: Navigation elements that point to specific documentation pages
- **Fields**:
  - label: string (display text, Title Case)
  - to: string (Docusaurus document ID path)
- **Validation**: 'to' path must resolve to a valid document in the docs directory

## Entity: Module Link
- **Name**: ModuleLink
- **Description**: Navigation elements that point to specific learning modules in sequential order
- **Fields**:
  - label: string (module name, Title Case)
  - to: string (Docusaurus document ID path for the module directory)
- **Validation**: 'to' path must resolve to a valid module directory in the docs directory
- **Ordering**: Must follow sequential module order (Module 1 → Module N)

## Entity: External Link
- **Name**: ExternalLink
- **Description**: Navigation elements that point to external resources
- **Fields**:
  - label: string (display text)
  - href: string (full external URL)
- **Validation**: 'href' must be a valid external URL

## State Transitions
- Pre-implementation: Footer contains default Docusaurus links (Docs, Community, More)
- Post-implementation: Footer contains custom links (Docs, Modules, More) as specified

## Constraints
- Footer MUST contain exactly three link groups: Docs, Modules, More
- Docs section MUST contain: Introduction, Setup Guide, Capstone, Conclusion in that order
- Modules section MUST contain all available modules in sequential order
- More section MUST contain: GitHub, References
- All links MUST resolve to valid destinations without broken-link warnings
