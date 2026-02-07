# Server Component Patterns

## Streaming with Suspense
```tsx
<Suspense fallback={<TaskListSkeleton />}>
  <TaskList tasks={tasks} />
</Suspense>
Parallel Data Fetching
tsxconst [tasks, analytics] = await Promise.all([
  db.task.findMany(),
  fetchAnalytics()
]);