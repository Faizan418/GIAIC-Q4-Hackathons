---
name: responsive-component-design
description: Designs reusable, mobile-first responsive components using Tailwind CSS with proper breakpoints, flexible layouts, grid/flex systems, and accessibility considerations.
---

# Responsive Component Design Skill

This skill creates beautiful, reusable components that look perfect on all devices.

## Mobile-First Philosophy
- Start with base (mobile) styles
- Enhance with sm:, md:, lg:, xl: for larger screens
- Use relative units (rem, %, viewport)

## Common Responsive Patterns

### 1. Responsive Grid
```tsx
<div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-6">
  {/* items */}
</div>
2. Stacking Layout
tsx<div className="flex flex-col md:flex-row gap-8 items-start">
  <aside className="w-full md:w-64">Sidebar</aside>
  <main className="flex-1">Content</main>
</div>
3. Responsive Typography
tsx<h1 className="text-3xl sm:text-4xl md:text-5xl lg:text-6xl font-bold">
  Responsive Title
</h1>
4. Card Component
tsx<div className="w-full max-w-sm mx-auto p-6 bg-card rounded-lg shadow-md border">
  <h3 className="text-xl font-semibold mb-2">Card Title</h3>
  <p className="text-muted-foreground">Description</p>
</div>
```