import { useState } from 'react';
import { Input } from '../ui/input';
import { Select, SelectTrigger, SelectValue, SelectContent, SelectItem } from '../ui/select';
import { Button } from '../ui/button';
import { Filter, X } from 'lucide-react';

interface TaskFiltersProps {
  filters: {
    status: 'completed' | 'pending' | undefined;
    priority: string;
    dueDate: string;
    search: string;
  };
  onFilterChange: (filters: TaskFiltersProps['filters']) => void;
}

export function TaskFilters({ filters, onFilterChange }: TaskFiltersProps) {
  const [showAdvanced, setShowAdvanced] = useState(false);

  const handleStatusChange = (value: string) => {
    let status: 'completed' | 'pending' | undefined;
    if (value === 'all') status = undefined;
    if (value === 'completed') status = 'completed';
    if (value === 'pending') status = 'pending';

    onFilterChange({ ...filters, status });
  };

  const handlePriorityChange = (value: string) => {
    onFilterChange({ ...filters, priority: value });
  };

  const handleSearchChange = (value: string) => {
    onFilterChange({ ...filters, search: value });
  };

  const handleClearFilters = () => {
    onFilterChange({
      status: undefined,
      priority: '',
      dueDate: '',
      search: ''
    });
  };

  return (
    <div className="space-y-4">
      <div className="flex flex-wrap gap-2 items-center">
        <Select
          value={
            filters.status === undefined ? 'all' :
            filters.status
          }
          onValueChange={handleStatusChange}
        >
          <SelectTrigger className="w-[180px]">
            <SelectValue placeholder="Status" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="all">All Tasks</SelectItem>
            <SelectItem value="pending">Pending</SelectItem>
            <SelectItem value="completed">Completed</SelectItem>
          </SelectContent>
        </Select>

        <Select
          value={filters.priority}
          onValueChange={handlePriorityChange}
        >
          <SelectTrigger className="w-[180px]">
            <SelectValue placeholder="Priority" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="">All Priorities</SelectItem>
            <SelectItem value="low">Low Priority</SelectItem>
            <SelectItem value="medium">Medium Priority</SelectItem>
            <SelectItem value="high">High Priority</SelectItem>
          </SelectContent>
        </Select>

        <Input
          placeholder="Search tasks..."
          value={filters.search}
          onChange={(e) => handleSearchChange(e.target.value)}
          className="max-w-xs"
        />

        <Button variant="outline" size="sm" onClick={() => setShowAdvanced(!showAdvanced)}>
          <Filter className="h-4 w-4 mr-2" />
          Filters
        </Button>

        {(filters.status !== undefined || filters.priority || filters.search) && (
          <Button variant="outline" size="sm" onClick={handleClearFilters}>
            <X className="h-4 w-4 mr-2" />
            Clear
          </Button>
        )}
      </div>

      {showAdvanced && (
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 p-4 bg-gray-50 dark:bg-gray-800/50 rounded-lg">
          <div>
            <label className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-1 block">
              Due Date
            </label>
            <Input
              type="date"
              value={filters.dueDate}
              onChange={(e) => onFilterChange({ ...filters, dueDate: e.target.value })}
            />
          </div>
        </div>
      )}
    </div>
  );
}