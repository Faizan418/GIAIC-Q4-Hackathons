import React from 'react';
import Link from '@docusaurus/Link';
import type { FeatureCard } from './types';
import styles from './styles.module.css';

interface FeatureCardProps {
  card: FeatureCard;
}

const FeatureCardComponent: React.FC<FeatureCardProps> = ({ card }) => {
  const { id, title, description, imageUrl, linkUrl, linkText } = card;

  if (linkUrl && linkText) {
    // If there's a link, make the entire card clickable
    return (
      <Link
        to={linkUrl}
        className={styles.featureCard}
        id={id}
        role="region"
        aria-label={`Learn more about ${title}`}
        tabIndex={0}
      >
        <div className={styles.featureImage}>
          <img
            src={imageUrl}
            alt={`${title} illustration`}
            loading="lazy"
            onError={(e) => {
              // Fallback if image fails to load
              e.currentTarget.style.display = 'none';
            }}
          />
        </div>
        <h3 id={`${id}-title`} className={styles.featureTitle}>{title}</h3>
        <p className={styles.featureDescription}>{description}</p>
        <span className={styles.featureLink}>
          {linkText}
        </span>
      </Link>
    );
  } else {
    // If there's no link, render as a static card
    return (
      <div
        className={styles.featureCard}
        id={id}
        role="region"
        aria-labelledby={`${id}-title`}
        tabIndex={0}
      >
        <div className={styles.featureImage}>
          <img
            src={imageUrl}
            alt={`${title} illustration`}
            loading="lazy"
            onError={(e) => {
              // Fallback if image fails to load
              e.currentTarget.style.display = 'none';
            }}
          />
        </div>
        <h3 id={`${id}-title`} className={styles.featureTitle}>{title}</h3>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    );
  }
};

export default FeatureCardComponent;
