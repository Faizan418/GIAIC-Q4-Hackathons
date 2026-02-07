---
name: client-component
description: Properly marks and implements Client Components for interactivity, state management, and browser APIs while keeping bundle size minimal.
---

# Client Component Skill

Use "use client" only when necessary.

## When to Use Client Components
- useState, useEffect
- Event handlers (onClick, onChange)
- Browser APIs (localStorage, geolocation)
- Interactive UI (modals, drag-drop)

## Pattern: Push Interactivity Down
```tsx
// app/tasks/page.tsx (Server)
import TaskList from '@/components/TaskList';

export default async function TasksPage() {
  const tasks = await getTasks();
  return <TaskList initialTasks={tasks} />;
}

// components/TaskList.tsx (Client)
'use client';

export default function TaskList({ initialTasks }) {
  const [tasks, setTasks] = useState(initialTasks);
  // interactivity here
}