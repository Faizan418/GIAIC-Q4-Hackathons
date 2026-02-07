# Homepage Feature Cards

This directory contains components for displaying feature cards on the homepage.

## Components

### FeatureCard
A single feature card component that displays:
- Title
- Description
- SVG image
- Optional navigation link

### HomepageFeatureCards
A container component that displays multiple FeatureCard components in a responsive grid layout.

## Usage

The components are used on the homepage (`src/pages/index.tsx`) to replace the default Docusaurus feature section.

## Data Configuration

Feature card data is defined in `data.ts` with the following structure:
```typescript
{
  id: string,
  title: string,
  description: string,
  imageUrl: string,
  linkUrl?: string,
  linkText?: string,
  order: number
}
```

## Styling

Styling is handled through CSS modules in `styles.module.css` with responsive design and accessibility features included.
