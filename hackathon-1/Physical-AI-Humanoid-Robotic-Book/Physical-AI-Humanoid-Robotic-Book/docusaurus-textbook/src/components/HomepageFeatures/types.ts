export interface FeatureCard {
  id: string;
  title: string;
  description: string;
  imageUrl: string;
  linkUrl?: string;
  linkText?: string;
  order: number;
}

export interface CardLayout {
  cards: FeatureCard[];
  layoutType?: 'grid' | 'carousel';
  maxCardsPerRow?: number;
}
