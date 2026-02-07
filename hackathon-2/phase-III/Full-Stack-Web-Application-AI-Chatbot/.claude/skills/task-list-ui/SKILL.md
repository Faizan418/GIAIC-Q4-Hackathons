---
name: task-list-ui
description: Implements interactive task/todo list UI with add, complete, edit, delete, filtering, sorting, and persistence features.
---

# Task List UI Skill

This skill builds a fully interactive task management interface.

## Core Features

- Add new task
- Mark as complete/incomplete
- Edit task title
- Delete task
- Filter (all/active/completed)
- Sort (date, priority)
- Persistence (localStorage or backend)

## Component Structure

components/
└── task-list/
├── TaskItem.tsx
├── TaskForm.tsx
├── TaskFilters.tsx
└── TaskList.tsx
text## TaskItem Example

```tsx
function TaskItem({ task, onToggle, onDelete, onEdit }) {
  return (
    <div className="flex items-center gap-4 p-4 bg-card rounded-lg">
      <input
        type="checkbox"
        checked={task.completed}
        onChange={() => onToggle(task.id)}
        className="w-5 h-5"
      />
      <span className={task.completed ? "line-through text-muted" : ""}>
        {task.title}
      </span>
      <button onClick={() => onEdit(task)} className="ml-auto">Edit</button>
      <button onClick={() => onDelete(task.id)} className="text-red-500">Delete</button>
    </div>
  );
}
