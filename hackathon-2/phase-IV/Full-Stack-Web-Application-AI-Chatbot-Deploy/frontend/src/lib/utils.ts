import { type ClassValue, clsx } from "clsx";
import { twMerge } from "tailwind-merge";
import { format, isAfter, isToday, isTomorrow } from 'date-fns';

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

/**
 * Formats a date string to a human-readable format
 */
export function formatDate(dateString: string): string {
  const date = new Date(dateString);
  return format(date, 'MMM dd, yyyy');
}

/**
 * Returns relative time string for a given date
 * Examples: "Today", "Tomorrow", "Yesterday", "in 2 days", "Jan 15, 2024"
 */
export function getRelativeTime(dateString: string): string {
  const date = new Date(dateString);
  const now = new Date();

  if (isToday(date)) {
    return 'Today';
  }

  if (isTomorrow(date)) {
    return 'Tomorrow';
  }

  if (isAfter(now, date)) {
    // Date is in the past
    const diffDays = Math.floor((now.getTime() - date.getTime()) / (1000 * 60 * 60 * 24));
    if (diffDays === 1) {
      return 'Yesterday';
    } else if (diffDays < 7) {
      return `${diffDays} days ago`;
    } else {
      return format(date, 'MMM dd');
    }
  } else {
    // Date is in the future
    const diffDays = Math.floor((date.getTime() - now.getTime()) / (1000 * 60 * 60 * 24));
    if (diffDays === 1) {
      return 'Tomorrow';
    } else if (diffDays < 7) {
      return `in ${diffDays} days`;
    } else {
      return format(date, 'MMM dd');
    }
  }
}

/**
 * Checks if a date is overdue (past due and not completed)
 */
export function isOverdue(dateString: string | undefined): boolean {
  if (!dateString) return false;

  const dueDate = new Date(dateString);
  const now = new Date();
  return isAfter(now, dueDate);
}

/**
 * Validates task title length (1-200 characters)
 */
export function validateTaskTitle(title: string): boolean {
  const trimmedTitle = title.trim();
  return trimmedTitle.length >= 1 && trimmedTitle.length <= 200;
}

/**
 * Validates task description length (0-1000 characters)
 */
export function validateTaskDescription(description?: string): boolean {
  if (!description) return true;
  return description.length <= 1000;
}

/**
 * Validates tags array (max 10 tags, each 1-30 characters)
 */
export function validateTaskTags(tags: string[]): boolean {
  if (tags.length > 10) return false;

  for (const tag of tags) {
    const trimmedTag = tag.trim();
    if (trimmedTag.length < 1 || trimmedTag.length > 30) {
      return false;
    }
  }

  return true;
}

/**
 * Validates recurrence settings
 */
export function validateRecurrence(recurrence?: {
  type: string;
  interval?: number;
  endDate?: string;
  occurrences?: number;
}): boolean {
  if (!recurrence) return true;

  // If type is 'none', no other fields should be set
  if (recurrence.type === 'none') {
    return !recurrence.interval && !recurrence.endDate && !recurrence.occurrences;
  }

  // If type is not 'none', interval must be >= 1
  if (recurrence.interval !== undefined && recurrence.interval < 1) {
    return false;
  }

  // endDate and occurrences cannot both be set
  if (recurrence.endDate && recurrence.occurrences) {
    return false;
  }

  return true;
}