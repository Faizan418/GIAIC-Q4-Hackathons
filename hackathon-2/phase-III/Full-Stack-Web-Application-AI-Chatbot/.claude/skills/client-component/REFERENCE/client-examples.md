# Client Component Best Practices

## Minimal Client Boundary
```tsx
// Keep parent as Server Component
<ServerPage>
  <ClientInteractivePart />
</ServerPage>
State Management
tsx'use client';

import { useOptimistic } from 'react';

function TaskItem({ task }) {
  const [optimisticCompleted, toggleCompleted] = useOptimistic(
    task.completed,
    (current, optimisticValue) => optimisticValue
  );
  
  return (
    <input
      type="checkbox"
      checked={optimisticCompleted}
      onChange={toggleCompleted}
    />
  );
}# Client Component Best Practices

## Minimal Client Boundary
```tsx
// Keep parent as Server Component
<ServerPage>
  <ClientInteractivePart />
</ServerPage>
State Management
tsx'use client';

import { useOptimistic } from 'react';

function TaskItem({ task }) {
  const [optimisticCompleted, toggleCompleted] = useOptimistic(
    task.completed,
    (current, optimisticValue) => optimisticValue
  );
  
  return (
    <input
      type="checkbox"
      checked={optimisticCompleted}
      onChange={toggleCompleted}
    />
  );
}