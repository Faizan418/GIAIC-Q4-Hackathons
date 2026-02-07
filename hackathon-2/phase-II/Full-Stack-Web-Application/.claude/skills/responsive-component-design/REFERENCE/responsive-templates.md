# Reusable Responsive Components

## Dashboard Layout
```tsx
<div className="min-h-screen flex flex-col lg:flex-row">
  <aside className="w-full lg:w-64 bg-sidebar">Sidebar</aside>
  <main className="flex-1 p-4 lg:p-8">Main Content</main>
</div>
Image Gallery
tsx<div className="grid grid-cols-2 sm:grid-cols-3 md:grid-cols-4 gap-4">
  <img className="w-full aspect-square object-cover rounded-lg" src="..." />
</div>
Navigation Bar
tsx<nav className="flex flex-col sm:flex-row items-center justify-between gap-4 p-4">
  <div className="flex items-center gap-8">
    <Logo />
    <Links className="hidden sm:flex" />
  </div>
  <MobileMenu className="sm:hidden" />
</nav>