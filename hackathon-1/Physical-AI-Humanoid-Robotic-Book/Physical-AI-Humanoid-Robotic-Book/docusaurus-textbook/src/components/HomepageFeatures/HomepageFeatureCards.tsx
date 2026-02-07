import React from 'react';
import type { CardLayout } from './types';
import FeatureCard from './FeatureCard';
import styles from './styles.module.css';

interface HomepageFeatureCardsProps {
  layout?: CardLayout;
}

const HomepageFeatureCards: React.FC<HomepageFeatureCardsProps> = ({
  layout = {
    cards: require('./data').featureCardsData,
    layoutType: 'grid',
    maxCardsPerRow: 3
  }
}) => {
  const { cards, layoutType = 'grid', maxCardsPerRow = 3 } = layout;

  return (
    <section
      className={styles.featuresSection}
      aria-labelledby="features-title"
      role="region"
    >
      <div className={styles.container}>
        <h2 id="features-title" className={styles.sectionTitle} tabIndex={-1}>
          Key Features
        </h2>
        <div
          className={styles.grid}
          style={{ gridTemplateColumns: `repeat(${maxCardsPerRow}, 1fr)` }}
          role="list"
        >
          {cards.map((card) => (
            <div role="listitem" key={card.id}>
              <FeatureCard card={card} />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default HomepageFeatureCards;
