# Server vs Client Component Guide

## Server Component (Default)
```tsx
// No "use client"
// Can await promises directly
// Access server-only modules (fs, db)
// No interactivity
async function Page() {
  const data = await fetchFromDB();
  return <div>{data}</div>;
}
Client Component
tsx'use client';

import { useState } from 'react';

export default function Counter() {
  const [count, setCount] = useState(0);
  return <button onClick={() => setCount(c => c + 1)}>{count}</button>;
}
Best Practice: Push interactivity down

Keep pages/layouts as Server Components
Extract interactive parts to Client Components
Pass data as props