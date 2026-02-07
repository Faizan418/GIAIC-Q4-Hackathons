# Reusable Tailwind Component Classes

## Card Component
```tsx
<div className="rounded-lg border bg-card text-card-foreground shadow-sm">
  <div className="p-6">
    <h3 className="text-2xl font-semibold">Title</h3>
    <p className="mt-2 text-sm text-muted-foreground">Description</p>
  </div>
</div>
Responsive Grid
tsx<div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
  {/* items */}
</div>
Dark Mode Example
tsx<body className="min-h-screen bg-background text-foreground dark">
Utility Best Practices

Use arbitrary values sparingly
Prefer meaningful class names order (layout → sizing → typography → colors)
Extract repeated patterns to component classes