# Complete CRUD Example

## FastAPI Endpoint (Create)
```python
@router.post("/tasks", response_model=TaskRead)
async def create_task(task: TaskCreate, current_user: User = Depends(get_current_user)):
    return await crud.create_task(db, task, current_user.id)
Next.js Server Action (Delete)
tsx'use server';

import { revalidatePath } from 'next/cache';

export async function deleteTask(id: number) {
  const res = await fetch(`/api/tasks/${id}`, { method: 'DELETE' });
  if (!res.ok) throw new Error('Failed');
  revalidatePath('/tasks');
}
Client Component Usage
tsx'use client';

import { deleteTask } from '@/actions/tasks';

function TaskItem({ task }) {
  return (
    <button
      onClick={() => deleteTask(task.id)}
      className="text-red-500"
    >
      Delete
    </button>
  );
}