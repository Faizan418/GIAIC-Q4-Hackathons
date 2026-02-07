---
name: tailwind-css-styling
description: Implements clean, consistent, reusable Tailwind CSS styling with design tokens from spec-kit, dark mode support, custom components, and utility best practices.
---

# Tailwind CSS Styling Skill

This skill creates beautiful, maintainable UI using Tailwind with strong conventions.

## Core Principles
- Use spec-kit tokens for colors, spacing, radii
- Prefer utility classes over custom CSS
- Create reusable component classes (e.g., btn-primary)
- Full dark mode support
- Responsive first (mobile → desktop)

## tailwind.config.ts Integration with spec-kit
```ts
// tailwind.config.ts
import { tokens } from '@spec-kit';

export default {
  darkMode: 'class',
  content: [
    './app/**/*.{ts,tsx}',
    './components/**/*.{ts,tsx}',
    './packages/ui/**/*.{ts,tsx}'
  ],
  theme: {
    extend: {
      colors: tokens.colors,
      spacing: tokens.spacing,
      borderRadius: tokens.radii,
      fontFamily: {
        sans: [tokens.typography.fontFamily]
      }
    }
  }
}
Component Pattern
tsx// packages/ui/src/components/button.tsx
const buttonVariants = cva(
  "inline-flex items-center justify-center rounded-md font-medium transition-colors",
  {
    variants: {
      variant: {
        primary: "bg-primary text-primary-foreground hover:bg-primary/90",
        secondary: "bg-secondary text-secondary-foreground hover:bg-secondary/80",
        ghost: "hover:bg-accent hover:text-accent-foreground"
      },
      size: {
        sm: "h-9 px-3 text-sm",
        md: "h-10 px-4",
        lg: "h-12 px-6 text-lg"
      }
    },
    defaultVariants: {
      variant: "primary",
      size: "md"
    }
  }
);