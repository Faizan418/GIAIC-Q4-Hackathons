---
name: server-components
description: Leverages React Server Components for optimal performance by fetching data on the server, reducing client bundle size, and enabling streaming.
---

# Server Components Skill

This skill maximizes Next.js performance using Server Components (default in App Router).

## When to Use Server Components
- Data fetching (DB, API)
- Sensitive data (secrets, user info)
- Large dependency trees
- Static content

## Conversion Rules
- Default: Server Component
- Add "use client" only when needed (state, effects, browser APIs)

## Example Pattern
```tsx
// app/dashboard/page.tsx - Server Component by default
import { db } from '@/lib/db';

export default async function Dashboard() {
  const user = await getCurrentUser(); // server-side
  const posts = await db.post.findMany({ where: { userId: user.id } });

  return (
    <>
      <h1>Welcome {user.name}</h1>
      <PostList posts={posts} /> {/* can be Client if interactive */}
    </>
  );
}
Streaming with Suspense
tsx<Suspense fallback={<Loading />}>
  <HeavyComponent />
</Suspense>