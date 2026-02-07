# Responsive Component Templates

## Navigation
```tsx
<nav className="flex flex-col sm:flex-row gap-4 sm:gap-8">
  <Link href="/">Home</Link>
  <Link href="/about">About</Link>
</nav>
Hero Section
tsx<section className="px-6 py-12 sm:py-24 text-center">
  <h1 className="text-4xl sm:text-5xl md:text-6xl font-bold">
    Welcome
  </h1>
  <p className="mt-4 text-lg sm:text-xl text-muted-foreground max-w-2xl mx-auto">
    Description
  </p>
</section>
Mobile Menu Pattern
tsx<button
  className="md:hidden"
  onClick={() => setOpen(!open)}
>
  Menu
</button>
{open && (
  <div className="absolute inset-x-0 top-full bg-background shadow-lg md:hidden">
    {/* links */}
  </div>
)}