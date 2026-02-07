# Client-Side Fetching Templates

## Server Action Mutation
```tsx
'use server';

import { revalidatePath } from 'next/cache';

export async function createPost(formData: FormData) {
  // authenticated automatically via cookies
  await db.post.create({ data: { ... } });
  revalidatePath('/posts');
}
Protected Client Fetch
tsxfetch('/api/protected', {
  credentials: 'include', // sends HttpOnly refresh cookie
  headers: {
    'Content-Type': 'application/json'
  }
})
Error Boundary Pattern
tsx<ErrorBoundary fallback={<ErrorMessage />}>
  <Suspense fallback={<Loading />}>
    <DashboardData />
  </Suspense>
</ErrorBoundary>