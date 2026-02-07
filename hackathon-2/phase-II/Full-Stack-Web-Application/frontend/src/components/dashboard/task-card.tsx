// frontend/components/dashboard/task-card.tsx
'use client';

import { Task } from '@/types/task';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import { Checkbox } from '@/components/ui/checkbox';
import { formatDate, isOverdue } from '@/lib/utils';
import { useState } from 'react';

interface TaskCardProps {
  task: Task;
  onToggleComplete: (id: string, completed: boolean) => void;
  onEdit: (task: Task) => void;
  onDelete: (id: string) => void;
}

export function TaskCard({ task, onToggleComplete, onEdit, onDelete }: TaskCardProps) {
  const [showConfirmDelete, setShowConfirmDelete] = useState(false);

  const handleDeleteClick = () => {
    if (showConfirmDelete) {
      onDelete(task.id);
      setShowConfirmDelete(false);
    } else {
      setShowConfirmDelete(true);
      // Hide the confirmation after 3 seconds
      setTimeout(() => setShowConfirmDelete(false), 3000);
    }
  };

  const handleDeleteCancel = () => {
    setShowConfirmDelete(false);
  };

  // Check if task is overdue but not completed
  const isTaskOverdue = task.due_date && isOverdue(task.due_date) && !task.is_completed;

  return (
    <div className={`border rounded-lg p-4 hover:shadow-md transition-shadow ${isTaskOverdue ? 'border-red-500 bg-red-50 dark:bg-red-900/20' : ''}`}>
      <div className="flex items-start gap-3">
        <Checkbox
          checked={task.is_completed}
          onCheckedChange={(checked) => onToggleComplete(task.id, Boolean(checked))}
          className="mt-1"
        />

        <div className="flex-1 min-w-0">
          <h3 className={`font-medium ${task.is_completed ? 'line-through text-muted-foreground' : ''}`}>
            {task.title}
          </h3>

          {task.description && (
            <p className="text-sm text-muted-foreground mt-1">
              {task.description}
            </p>
          )}

          <div className="flex flex-wrap items-center gap-2 mt-2">
            <Badge
              variant={task.priority === 'HIGH' ? 'destructive' :
                      task.priority === 'MEDIUM' ? 'default' : 'secondary'}
            >
              {task.priority.toLowerCase()}
            </Badge>

            {task.due_date && (
              <div className={`text-xs flex items-center ${isOverdue(task.due_date) && !task.is_completed ? 'text-red-600 font-bold' : 'text-muted-foreground'}`}>
                <span className="mr-1">📅</span>
                {formatDate(task.due_date)}
              </div>
            )}

          </div>
        </div>

        <div className="flex gap-1">
          <Button variant="ghost" size="sm" onClick={() => onEdit(task)}>
            ✏️
          </Button>
          <Button
            variant="ghost"
            size="sm"
            onClick={showConfirmDelete ? handleDeleteCancel : handleDeleteClick}
            className={showConfirmDelete ? "text-green-600 hover:text-green-700" : "text-destructive hover:text-destructive"}
          >
            {showConfirmDelete ? "✓ Cancel" : "🗑️"}
          </Button>
        </div>
      </div>

      {showConfirmDelete && (
        <div className="mt-3 text-sm text-destructive">
          Are you sure? Click again to delete.
        </div>
      )}
    </div>
  );
}