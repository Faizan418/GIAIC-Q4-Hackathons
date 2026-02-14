---
name: server-component
description: Maximizes use of React Server Components for data fetching, reduced bundle size, and improved performance in Next.js App Router.
---

# Server Component Skill

Server Components are default in App Router – use them aggressively.

## When to Use Server Components
- Initial data loading
- Database queries
- Sensitive data
- Static content
- Large dependencies

## Example: Dashboard Page
```tsx
// app/dashboard/page.tsx (Server Component by default)
import { db } from '@/lib/db';

export default async function Dashboard() {
  const user = await getCurrentUser(); // server-side
  const tasks = await db.task.findMany({
    where: { userId: user.id }
  });

  return (
    <div>
      <h1>Welcome back, {user.name}</h1>
      <TaskList tasks={tasks} /> {/* Client if interactive */}
    </div>
  );
}