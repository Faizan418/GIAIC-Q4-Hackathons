# Quickstart: Homepage Feature Cards Implementation

## Prerequisites
- Node.js 18+ installed
- Docusaurus project already set up
- Basic knowledge of React and TypeScript

## Setup Steps

### 1. Create Component Directory
```bash
mkdir -p src/components/HomepageFeatures
```

### 2. Create Feature Card Component
Create `src/components/HomepageFeatures/FeatureCard.tsx`:
```tsx
import React from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './styles.module.css';

interface FeatureCardProps {
  title: string;
  description: string;
  imageUrl: string;
  linkUrl?: string;
  linkText?: string;
}

export default function FeatureCard({
  title,
  description,
  imageUrl,
  linkUrl,
  linkText,
}: FeatureCardProps): JSX.Element {
  return (
    <div className={styles.featureCard}>
      <div className={styles.imageContainer}>
        <img src={imageUrl} alt={title} className={styles.featureImage} />
      </div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
      {linkUrl && linkText && (
        <Link to={linkUrl} className={styles.featureLink}>
          {linkText}
        </Link>
      )}
    </div>
  );
}
```

### 3. Create Main Cards Component
Create `src/components/HomepageFeatures/HomepageFeatureCards.tsx`:
```tsx
import React from 'react';
import clsx from 'clsx';
import FeatureCard from './FeatureCard';
import styles from './styles.module.css';

interface HomepageFeatureCardsProps {
  cards: Array<{
    title: string;
    description: string;
    imageUrl: string;
    linkUrl?: string;
    linkText?: string;
  }>;
}

export default function HomepageFeatureCards({
  cards,
}: HomepageFeatureCardsProps): JSX.Element {
  return (
    <section className={styles.featuresSection}>
      <div className={styles.container}>
        <div className={styles.grid}>
          {cards.map((card, index) => (
            <FeatureCard
              key={index}
              title={card.title}
              description={card.description}
              imageUrl={card.imageUrl}
              linkUrl={card.linkUrl}
              linkText={card.linkText}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
```

### 4. Create Styles Module
Create `src/components/HomepageFeatures/styles.module.css`:
```css
.featuresSection {
  padding: 4rem 0;
  width: 100%;
}

.container {
  margin: 0 auto;
  padding: 0 1rem;
  max-width: 1200px;
}

.grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 2rem;
  align-items: start;
}

.featureCard {
  background: var(--ifm-card-background-color, #fff);
  border-radius: 8px;
  padding: 1.5rem;
  text-align: center;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  transition: transform 0.2s ease, box-shadow 0.2s ease;
}

.featureCard:hover {
  transform: translateY(-5px);
  box-shadow: 0 8px 15px rgba(0, 0, 0, 0.15);
}

.imageContainer {
  margin-bottom: 1rem;
  display: flex;
  justify-content: center;
}

.featureImage {
  max-width: 100%;
  height: auto;
  max-height: 120px;
}

.featureTitle {
  margin-bottom: 0.75rem;
  font-size: 1.25rem;
  font-weight: 600;
  color: var(--ifm-heading-color);
}

.featureDescription {
  margin-bottom: 1rem;
  color: var(--ifm-font-color-base);
  line-height: 1.6;
}

.featureLink {
  display: inline-block;
  padding: 0.5rem 1rem;
  background-color: var(--ifm-color-primary);
  color: white;
  border-radius: 4px;
  text-decoration: none;
  transition: background-color 0.2s ease;
}

.featureLink:hover {
  background-color: var(--ifm-color-primary-dark);
}

/* Responsive adjustments */
@media (max-width: 996px) {
  .grid {
    grid-template-columns: repeat(2, 1fr);
  }
}

@media (max-width: 768px) {
  .featuresSection {
    padding: 2rem 0;
  }

  .grid {
    grid-template-columns: 1fr;
  }
}
```

### 5. Update Homepage
In `src/pages/index.tsx`, replace the existing feature section with:
```tsx
import HomepageFeatureCards from '@site/src/components/HomepageFeatures/HomepageFeatureCards';

// Define your feature cards data
const featureCards = [
  {
    title: 'Feature One',
    description: 'Description of the first feature',
    imageUrl: '/img/feature-1.svg',
    linkUrl: '/docs/category/tutorials',
    linkText: 'Learn More',
  },
  {
    title: 'Feature Two',
    description: 'Description of the second feature',
    imageUrl: '/img/feature-2.svg',
    linkUrl: '/docs/category/tutorials',
    linkText: 'Learn More',
  },
  {
    title: 'Feature Three',
    description: 'Description of the third feature',
    imageUrl: '/img/feature-3.svg',
    linkUrl: '/docs/category/tutorials',
    linkText: 'Learn More',
  },
];

// Use the component in your JSX
<HomepageFeatureCards cards={featureCards} />
```

### 6. Add SVG Images
Place your SVG images in the `static/img/` directory and reference them in the component.

### 7. Run Development Server
```bash
npm run start
```

## Testing Checklist
- [ ] Cards display properly on desktop
- [ ] Cards stack properly on mobile
- [ ] Hover effects work
- [ ] Links navigate correctly
- [ ] Images load properly
- [ ] Contrast ratio meets 4.5:1 minimum
- [ ] Keyboard navigation works
- [ ] Screen readers can interpret content
