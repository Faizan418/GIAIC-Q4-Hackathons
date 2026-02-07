# Data Model: Homepage Feature Cards Upgrade

## Feature Card Entity

### Properties
- **id**: string - Unique identifier for the card
- **title**: string - Feature title displayed prominently
- **description**: string - Brief description of the feature
- **imageUrl**: string - Path to the SVG/PNG image asset
- **linkUrl**: string? - Optional URL for card navigation (nullable)
- **linkText**: string? - Optional text for the navigation link (nullable)
- **order**: number - Display order for the card in the grid

### Validation Rules
- title: Required, minimum 1 character, maximum 100 characters
- description: Required, minimum 1 character, maximum 300 characters
- imageUrl: Required, must be a valid path to an image asset
- order: Required, must be a positive integer

## Card Configuration

### Structure
```typescript
interface FeatureCard {
  id: string;
  title: string;
  description: string;
  imageUrl: string;
  linkUrl?: string;
  linkText?: string;
  order: number;
}
```

## Card Layout Entity

### Properties
- **cards**: FeatureCard[] - Array of feature cards to display
- **layoutType**: 'grid' | 'carousel' - How to arrange the cards
- **maxCardsPerRow**: number - Maximum number of cards per row (responsive)

### Validation Rules
- cards: Required array with minimum 1 and maximum 6 cards
- layoutType: Required, defaults to 'grid'
- maxCardsPerRow: Required, must be between 1 and 4

## State Transitions

### Hover State
- **Trigger**: Mouse hover or focus on card
- **Effect**: Slight elevation, shadow, or color change to indicate interactivity

### Active State
- **Trigger**: Card is clicked (if navigable)
- **Effect**: Visual feedback before navigation occurs
