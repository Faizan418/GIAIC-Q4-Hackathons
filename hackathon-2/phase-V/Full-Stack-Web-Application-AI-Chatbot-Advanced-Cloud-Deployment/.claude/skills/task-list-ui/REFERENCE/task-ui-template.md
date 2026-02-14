# Task List Implementation

## TaskForm Component
```tsx
'use client';

import { useState } from 'react';

export function TaskForm({ onAdd }) {
  const [title, setTitle] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (title.trim()) {
      onAdd({ title, completed: false });
      setTitle('');
    }
  };

  return (
    <form onSubmit={handleSubmit} className="flex gap-2 mb-8">
      <input
        type="text"
        value={title}
        onChange={(e) => setTitle(e.target.value)}
        placeholder="Add a new task..."
        className="flex-1 px-4 py-2 border rounded-lg"
      />
      <button type="submit" className="px-6 py-2 bg-primary text-white rounded-lg">
        Add
      </button>
    </form>
  );
}
Filters
tsx<div className="flex gap-4 mb-6">
  <button className={filter === 'all' ? 'font-bold' : ''}>All</button>
  <button className={filter === 'active' ? 'font-bold' : ''}>Active</button>
  <button className={filter === 'completed' ? 'font-bold' : ''}>Completed</button>
</div>
```