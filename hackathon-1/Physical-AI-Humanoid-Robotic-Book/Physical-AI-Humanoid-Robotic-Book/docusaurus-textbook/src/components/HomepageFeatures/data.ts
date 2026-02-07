import type { FeatureCard } from './types';

export const featureCardsData: FeatureCard[] = [
  {
    id: 'feature-1',
    title: 'Physical AI Integration',
    description: 'Learn how to bridge the digital brain with the physical form using advanced AI techniques that enable robots to learn, move, and intelligently perform complex tasks.',
    imageUrl: '/img/feature-1.svg',
    linkUrl: '/docs/Introduction',
    linkText: 'Explore Tutorials',
    order: 1
  },
  {
    id: 'feature-2',
    title: 'ROS 2 & NVIDIA Isaac',
    description: 'Master ROS 2 fundamentals and NVIDIA Isaac platform to create sophisticated robotic systems that bring simulations to life in real-world applications.',
    imageUrl: '/img/feature-2.svg',
    linkUrl: '/docs/Setup-Guide',
    linkText: 'Explore Tutorials',
    order: 2
  },
  {
    id: 'feature-3',
    title: 'Embodied Intelligence',
    description: 'Discover perception, planning, and action systems that transform knowledge into intelligent, autonomous humanoids capable of thriving in the real world.',
    imageUrl: '/img/feature-3.svg',
    linkUrl: '/docs/Conclusion',
    linkText: 'Explore Tutorials',
    order: 3
  }
];
