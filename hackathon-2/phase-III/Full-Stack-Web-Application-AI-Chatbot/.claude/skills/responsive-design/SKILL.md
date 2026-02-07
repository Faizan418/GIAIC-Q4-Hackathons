---
name: responsive-design
description: Implements mobile-first responsive design using Tailwind's responsive utilities, proper breakpoints, flexible layouts, and accessibility considerations.
---

# Responsive Design Skill

This skill ensures perfect experience on mobile, tablet, and desktop.

## Mobile-First Approach
- Base styles: mobile
- Use md:, lg:, xl: for larger screens
- Never use min-width media queries in custom CSS

## Standard Breakpoints (Tailwind Default)
- sm: 640px
- md: 768px
- lg: 1024px
- xl: 1280px
- 2xl: 1536px

## Common Patterns

**Responsive Grid**
```tsx
<div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
  {/* cards */}
</div>
Stacking Layout
tsx<div className="flex flex-col md:flex-row gap-8">
  <aside className="w-full md:w-64">Sidebar</aside>
  <main className="flex-1">Content</main>
</div>
Responsive Text
tsx<h1 className="text-2xl sm:text-3xl md:text-4xl lg:text-5xl font-bold">
  Responsive Heading
</h1>
Accessibility Tips

Touch targets ≥ 44px
Proper focus states
Reduced motion support
Screen reader friendly