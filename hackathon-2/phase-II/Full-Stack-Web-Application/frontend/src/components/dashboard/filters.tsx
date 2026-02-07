// frontend/components/dashboard/filters.tsx
'use client';

import { useState, useEffect } from 'react';
import { Input } from '@/components/ui/input';
import { Select, SelectTrigger, SelectValue, SelectContent, SelectItem } from '@/components/ui/select';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import { Task } from '@/types/task';

interface FiltersProps {
  onSearch: (query: string) => void;
  onFilterChange: (filters: {
    status: 'all' | 'completed' | 'pending';
    priority: 'all' | 'high' | 'medium' | 'low';
    tags: string[];
  }) => void;
  onSortChange: (sortBy: 'dueDate' | 'priority' | 'title' | 'created', sortOrder: 'asc' | 'desc') => void;
  availableTags: string[];
}

export function Filters({ onSearch, onFilterChange, onSortChange, availableTags }: FiltersProps) {
  const [searchQuery, setSearchQuery] = useState('');
  const [statusFilter, setStatusFilter] = useState<'all' | 'completed' | 'pending'>('all');
  const [priorityFilter, setPriorityFilter] = useState<'all' | 'high' | 'medium' | 'low'>('all');
  const [selectedTags, setSelectedTags] = useState<string[]>([]);
  const [sortBy, setSortBy] = useState<'dueDate' | 'priority' | 'title' | 'created'>('dueDate');
  const [sortOrder, setSortOrder] = useState<'asc' | 'desc'>('asc');
  const [showTagDropdown, setShowTagDropdown] = useState(false);

  // Debounced search
  useEffect(() => {
    const timer = setTimeout(() => {
      onSearch(searchQuery);
    }, 300);

    return () => clearTimeout(timer);
  }, [searchQuery, onSearch]);

  // Notify parent of filter changes
  useEffect(() => {
    onFilterChange({
      status: statusFilter,
      priority: priorityFilter,
      tags: selectedTags
    });
  }, [statusFilter, priorityFilter, selectedTags, onFilterChange]);

  // Notify parent of sort changes
  useEffect(() => {
    onSortChange(sortBy, sortOrder);
  }, [sortBy, sortOrder, onSortChange]);

  const handleTagSelect = (tag: string) => {
    if (!selectedTags.includes(tag)) {
      setSelectedTags([...selectedTags, tag]);
    }
    setShowTagDropdown(false);
  };

  const removeTag = (tag: string) => {
    setSelectedTags(selectedTags.filter(t => t !== tag));
  };

  return (
    <div className="space-y-4">
      <div className="flex flex-col md:flex-row gap-4">
        <div className="flex-1">
          <Input
            placeholder="Search tasks..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
          />
        </div>

        <div className="w-full md:w-32">
          <Select value={statusFilter} onValueChange={(value: 'all' | 'completed' | 'pending') => setStatusFilter(value)}>
            <SelectTrigger>
              <SelectValue placeholder="Status" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Tasks</SelectItem>
              <SelectItem value="pending">Pending</SelectItem>
              <SelectItem value="completed">Completed</SelectItem>
            </SelectContent>
          </Select>
        </div>

        <div className="w-full md:w-32">
          <Select value={priorityFilter} onValueChange={(value: 'all' | 'high' | 'medium' | 'low') => setPriorityFilter(value)}>
            <SelectTrigger>
              <SelectValue placeholder="Priority" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Priorities</SelectItem>
              <SelectItem value="high">High</SelectItem>
              <SelectItem value="medium">Medium</SelectItem>
              <SelectItem value="low">Low</SelectItem>
            </SelectContent>
          </Select>
        </div>

        <div className="w-full md:w-32 relative">
          <Select value={sortBy} onValueChange={(value: 'dueDate' | 'priority' | 'title' | 'created') => setSortBy(value as any)}>
            <SelectTrigger>
              <SelectValue placeholder="Sort by" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="dueDate">Due Date</SelectItem>
              <SelectItem value="priority">Priority</SelectItem>
              <SelectItem value="title">Title</SelectItem>
              <SelectItem value="created">Created</SelectItem>
            </SelectContent>
          </Select>
        </div>

        <div className="w-full md:w-24">
          <Select value={sortOrder} onValueChange={(value: 'asc' | 'desc') => setSortOrder(value)}>
            <SelectTrigger>
              <SelectValue placeholder="Order" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="asc">Ascending</SelectItem>
              <SelectItem value="desc">Descending</SelectItem>
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* Selected Tags */}
      <div className="flex flex-wrap gap-2">
        {selectedTags.map((tag) => (
          <Badge key={tag} variant="secondary" className="flex items-center gap-1">
            {tag}
            <Button
              variant="ghost"
              size="sm"
              className="h-4 w-4 p-0 ml-1"
              onClick={() => removeTag(tag)}
            >
              ×
            </Button>
          </Badge>
        ))}
      </div>

      {/* Available Tags Dropdown */}
      {availableTags.length > 0 && (
        <div className="relative">
          <Button
            type="button"
            variant="outline"
            className="w-full"
            onClick={() => setShowTagDropdown(!showTagDropdown)}
          >
            Add Tag Filter
          </Button>

          {showTagDropdown && (
            <div className="absolute z-10 mt-1 w-full bg-white dark:bg-gray-800 border rounded-md shadow-lg p-2 max-h-60 overflow-y-auto">
              {availableTags
                .filter(tag => !selectedTags.includes(tag))
                .map(tag => (
                  <Button
                    key={tag}
                    variant="ghost"
                    className="w-full justify-start"
                    onClick={() => handleTagSelect(tag)}
                  >
                    {tag}
                  </Button>
                ))}
            </div>
          )}
        </div>
      )}
    </div>
  );
}