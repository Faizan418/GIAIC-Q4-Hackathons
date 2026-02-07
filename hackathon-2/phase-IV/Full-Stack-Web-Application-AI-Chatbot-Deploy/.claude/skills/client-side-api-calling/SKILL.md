---
name: client-side-api-calling
description: Implements secure, efficient client-side data fetching using React Server Actions, SWR, or tanstack-query with proper error handling, loading states, authentication, and caching.
---

# Client-Side API Calling Skill

This skill handles data fetching in Client Components safely and efficiently.

## Preferred Methods (2026 Order)
1. React Server Actions (best for mutations + simple queries)
2. Route Handlers + fetch (for complex client needs)
3. tanstack-query (for advanced caching/stale-while-revalidate)

## Secure Fetching Pattern
```tsx
// app/dashboard/page.tsx (Client Component)
'use client';

import { useSession } from '@/hooks/use-session';

async function updateProfile(data: FormData) {
  "use server";
  // Server Action - runs on server with auth
  const session = await getSession();
  // ... update DB
}

export default function ProfileForm() {
  return (
    <form action={updateProfile}>
      {/* inputs */}
      <button type="submit">Save</button>
    </form>
  );
}
Client Fetch with Auth
tsxconst { data, error, isLoading } = useQuery({
  queryKey: ['user', userId],
  queryFn: async () => {
    const res = await fetch(`/api/users/${userId}`, {
      credentials: 'include' // for HttpOnly cookies
    });
    if (!res.ok) throw new Error('Failed');
    return res.json();
  }
});