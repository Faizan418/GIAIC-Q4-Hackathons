---
name: crud-route-implementation
description: Implements complete CRUD operations across Next.js routes and FastAPI endpoints with proper validation, error handling, and optimistic updates.
---

# CRUD Route Implementation Skill

Full Create-Read-Update-Delete flow with modern patterns.

## Next.js + FastAPI CRUD Flow

### 1. Create (POST)
```tsx
// Server Action
async function createTask(formData: FormData) {
  "use server";
  const res = await fetch(`${process.env.API_URL}/tasks`, {
    method: "POST",
    body: JSON.stringify(Object.fromEntries(formData)),
    headers: { "Content-Type": "application/json" }
  });
  revalidatePath('/tasks');
}
2. Read (GET)

Server Component fetches initial data
Client Component handles real-time updates

3. Update (PATCH)

Optimistic UI with useOptimistic
Server Action for actual update

4. Delete (DELETE)
tsxasync function deleteTask(id: number) {
  "use server";
  await fetch(`${process.env.API_URL}/tasks/${id}`, { method: "DELETE" });
  revalidatePath('/tasks');
}