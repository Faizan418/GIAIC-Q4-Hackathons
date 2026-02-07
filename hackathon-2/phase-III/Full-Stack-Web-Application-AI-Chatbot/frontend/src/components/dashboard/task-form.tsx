// frontend/components/dashboard/task-form.tsx
'use client';

import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { z } from 'zod';
import { Task } from '@/types/task';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Textarea } from '@/components/ui/textarea';
import { Select, SelectTrigger, SelectValue, SelectContent, SelectItem } from '@/components/ui/select';
import { Checkbox } from '@/components/ui/checkbox';
import { createTaskSchema, updateTaskSchema } from '@/lib/validation';
import { useState } from 'react';

// Use the updateTaskSchema for form validation since it allows partial fields
const taskFormSchema = updateTaskSchema.extend({
  title: z.string().min(1, 'Title is required').max(200),
});

type TaskFormValues = z.infer<typeof taskFormSchema>;

interface TaskFormProps {
  task?: Task;
  onSubmit: (data: TaskFormValues) => void;
  onCancel?: () => void;
}

export function TaskForm({ task, onSubmit, onCancel }: TaskFormProps) {
  const [tagsInput, setTagsInput] = useState('');
  const [showRecurrence, setShowRecurrence] = useState(false);

  const form = useForm<TaskFormValues>({
    resolver: zodResolver(taskFormSchema),
    defaultValues: {
      title: task?.title || '',
      description: task?.description || '',
      priority: task?.priority ? task.priority.toLowerCase() as 'low' | 'medium' | 'high' : 'medium',
      dueDate: task?.due_date || '',
      recurrence: { type: 'none' },
    }
  });

  const handleSubmit = (data: TaskFormValues) => {
    // Process tags from the input field
    const tags = tagsInput.split(',').map(tag => tag.trim()).filter(tag => tag);

    // Process recurrence
    const recurrence = showRecurrence && data.recurrence ? {
      type: data.recurrence.type || 'none',
      interval: data.recurrence.interval,
      endDate: data.recurrence.endDate,
      occurrences: data.recurrence.occurrences
    } : { type: 'none' as const };

    onSubmit({
      ...data,
      tags,
      recurrence
    });
  };

  return (
    <form onSubmit={form.handleSubmit(handleSubmit)} className="space-y-4">
      <div>
        <Input
          {...form.register('title')}
          placeholder="Task title"
          className={form.formState.errors.title ? 'border-destructive' : ''}
        />
        {form.formState.errors.title && (
          <p className="text-sm text-destructive mt-1">{form.formState.errors.title.message}</p>
        )}
      </div>

      <div>
        <Textarea
          {...form.register('description')}
          placeholder="Description (optional)"
        />
      </div>

      <div className="grid grid-cols-2 gap-4">
        <div>
          <Select
            value={form.watch('priority')}
            onValueChange={(value) => form.setValue('priority', value as any)}
          >
            <SelectTrigger>
              <SelectValue placeholder="Priority" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="low">Low</SelectItem>
              <SelectItem value="medium">Medium</SelectItem>
              <SelectItem value="high">High</SelectItem>
            </SelectContent>
          </Select>
        </div>

        <div>
          <Input
            type="date"
            {...form.register('dueDate')}
          />
        </div>
      </div>

      <div>
        <Input
          value={tagsInput}
          onChange={(e) => setTagsInput(e.target.value)}
          placeholder="Tags (comma separated)"
        />
      </div>

      {/* Recurrence Configuration */}
      <div className="space-y-2">
        <div className="flex items-center space-x-2">
          <Checkbox
            id="enable-recurrence"
            checked={showRecurrence}
            onCheckedChange={(checked) => setShowRecurrence(!!checked)}
          />
          <label htmlFor="enable-recurrence" className="text-sm font-medium leading-none peer-disabled:cursor-not-allowed peer-disabled:opacity-70">
            Enable Recurring Task
          </label>
        </div>

        {showRecurrence && (
          <div className="grid grid-cols-2 gap-4 p-4 border rounded-md">
            <div>
              <Select
                value={form.watch('recurrence.type') || 'none'}
                onValueChange={(value) => form.setValue('recurrence.type', value as any)}
              >
                <SelectTrigger>
                  <SelectValue placeholder="Recurrence type" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="daily">Daily</SelectItem>
                  <SelectItem value="weekly">Weekly</SelectItem>
                  <SelectItem value="monthly">Monthly</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div>
              <Input
                type="number"
                min="1"
                placeholder="Interval"
                {...form.register('recurrence.interval', { valueAsNumber: true })}
              />
            </div>

            <div>
              <Input
                type="date"
                placeholder="End date"
                {...form.register('recurrence.endDate')}
              />
            </div>

            <div>
              <Input
                type="number"
                min="1"
                placeholder="Occurrences"
                {...form.register('recurrence.occurrences', { valueAsNumber: true })}
              />
            </div>
          </div>
        )}
      </div>

      <div className="flex gap-2">
        <Button type="submit" className="flex-1">
          {task ? 'Update Task' : 'Add Task'}
        </Button>
        {onCancel && (
          <Button type="button" variant="outline" onClick={onCancel} className="flex-1">
            Cancel
          </Button>
        )}
      </div>
    </form>
  );
}