'use client';

import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, PieChart, Pie, Cell } from 'recharts';
import { useRouter } from 'next/navigation';
import { toast } from 'sonner';
import { dashboardApi, taskApi } from '@/lib/api';
import { Task } from '@/lib/api';

interface DashboardData {
  total_tasks: number;
  completed_tasks: number;
  pending_tasks: number;
  task_types: Record<string, number>;
  task_priorities: Record<string, number>;
}

interface DashboardStats {
  success: boolean;
  data: DashboardData;
}

export default function DashboardPage() {
  const [stats, setStats] = useState<DashboardData | null>(null);
  const [tasks, setTasks] = useState<Task[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [tasksLoading, setTasksLoading] = useState<boolean>(true);
  const [searchQuery, setSearchQuery] = useState<string>('');
  const [filterStatus, setFilterStatus] = useState<string>('');
  const [filterPriority, setFilterPriority] = useState<string>('');
  const router = useRouter();

  // Define consistent colors for charts
  const CHART_COLORS = ['#3b82f6', '#10b981', '#f59e0b', '#ef4444', '#8b5cf6', '#ec4899'];

  useEffect(() => {
    fetchDashboardStats();
    fetchTasks();
  }, []);

  // Effect to fetch tasks when filters change
  useEffect(() => {
    fetchTasks();
  }, [searchQuery, filterStatus, filterPriority]);

  const fetchDashboardStats = async () => {
    try {
      const response = await dashboardApi.getStats();
      // The response should have the structure { success: boolean, data: DashboardData }
      setStats(response.data);
    } catch (error) {
      console.error('Error fetching dashboard stats:', error);
      toast.error('Failed to load dashboard statistics');
      // Set empty stats to avoid showing error state but with real field names
      setStats({
        total_tasks: 0,
        completed_tasks: 0,
        pending_tasks: 0,
        task_types: { daily: 0, weekly: 0, monthly: 0 },
        task_priorities: { low: 0, medium: 0, high: 0 }
      });
    } finally {
      setLoading(false);
    }
  };

  const fetchTasks = async () => {
    try {
      setTasksLoading(true);

      // Fetch tasks from the API service with search and filter parameters
      const response = await taskApi.getTasks({
        ...(searchQuery && { search: searchQuery }),
        ...(filterStatus && { completed: filterStatus === 'completed' }), // Convert status to completed boolean for API
        ...(filterPriority && { priority: filterPriority })
      });

      // Extract tasks from the response structure - response could be an array or an object with data.tasks
      if (response && typeof response === 'object' && 'data' in response && response.data) {
        const responseData = response.data as { tasks?: Task[] };
        if (responseData && typeof responseData === 'object' && 'tasks' in responseData) {
          setTasks(responseData.tasks || []);
        } else {
          setTasks([]);
        }
      } else if (Array.isArray(response)) {
        setTasks(response);
      } else {
        setTasks([]);
      }
    } catch (error) {
      console.error('Error fetching tasks:', error);
      toast.error('Failed to load tasks');
      setTasks([]);
    } finally {
      setTasksLoading(false);
    }
  };

  if (loading) {
    return (
      <div className="min-h-screen bg-background py-4 sm:py-8">
        <div className="container mx-auto px-4 sm:px-6 lg:px-8 max-w-7xl">
          <div className="animate-pulse space-y-6">
            <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-6">
              {[1, 2, 3, 4].map((item) => (
                <div key={item} className="bg-card h-24 rounded-xl"></div>
              ))}
            </div>
            <div className="bg-card rounded-xl p-6">
              <div className="bg-muted h-64 rounded-lg"></div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  if (!stats) {
    return (
      <div className="min-h-screen bg-background py-4 sm:py-8">
        <div className="container mx-auto px-4 sm:px-6 lg:px-8 max-w-7xl">
          <Card className="bg-card">
            <CardHeader>
              <CardTitle>Error Loading Dashboard</CardTitle>
            </CardHeader>
            <CardContent>
              <p className="text-muted-foreground">Unable to load dashboard statistics. Please try again later.</p>
            </CardContent>
          </Card>
        </div>
      </div>
    );
  }

  // Prepare data for charts
  const typeChartData = Object.entries(stats.task_types).map(([name, value]) => ({
    name: name.charAt(0).toUpperCase() + name.slice(1),
    value,
  }));

  const priorityChartData = Object.entries(stats.task_priorities).map(([name, value]) => ({
    name: name.charAt(0).toUpperCase() + name.slice(1),
    value,
  }));

  // Colors for charts
  const COLORS = CHART_COLORS;

  return (
    <div className="min-h-screen bg-background py-4 sm:py-8 fade-in-up">
      <div className="container mx-auto px-4 sm:px-6 lg:px-8 max-w-7xl">
        <div className="mb-8 sm:mb-12 fade-in-up" style={{ animationDelay: '0.1s' }}>
          <h1 className="text-3xl sm:text-4xl font-bold text-foreground">Dashboard</h1>
          <p className="text-base sm:text-lg text-muted-foreground mt-2">Overview of your task statistics and productivity metrics</p>
        </div>

        {/* Stats Cards Grid */}
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4 sm:gap-6 mb-8 sm:mb-12">
          {/* Total Tasks Card */}
          <Card className="bg-card border border-border rounded-xl p-6 hover:shadow-lg hover-lift transition-all duration-300 cursor-pointer fade-in-up" style={{ animationDelay: '0.2s' }}>
            <CardHeader className="flex flex-row items-center justify-between pb-4">
              <div>
                <CardTitle className="text-sm font-medium text-muted-foreground">Total Tasks</CardTitle>
                <p className="text-2xl sm:text-3xl font-bold mt-2 text-foreground">{stats.total_tasks}</p>
              </div>
              <div className="p-3 rounded-full bg-blue-100 dark:bg-blue-900/50">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6 text-blue-600 dark:text-blue-400" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z" />
                  <polyline points="14 2 14 8 20 8" />
                  <line x1="16" y1="13" x2="8" y2="13" />
                  <line x1="16" y1="17" x2="8" y2="17" />
                  <polyline points="10 9 9 9 8 9" />
                </svg>
              </div>
            </CardHeader>
            <CardContent>
              <p className="text-xs text-muted-foreground">All tasks created</p>
            </CardContent>
          </Card>

          {/* Completed Tasks Card */}
          <Card className="bg-card border border-border rounded-xl p-6 hover:shadow-lg hover-lift transition-all duration-300 cursor-pointer fade-in-up" style={{ animationDelay: '0.3s' }}>
            <CardHeader className="flex flex-row items-center justify-between pb-4">
              <div>
                <CardTitle className="text-sm font-medium text-muted-foreground">Completed</CardTitle>
                <p className="text-2xl sm:text-3xl font-bold mt-2 text-green-600 dark:text-green-400">{stats.completed_tasks}</p>
              </div>
              <div className="p-3 rounded-full bg-green-100 dark:bg-green-900/50">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6 text-green-600 dark:text-green-400" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" />
                  <polyline points="22 4 12 14.01 9 11.01" />
                </svg>
              </div>
            </CardHeader>
            <CardContent>
              <p className="text-xs text-muted-foreground">Tasks completed</p>
            </CardContent>
          </Card>

          {/* Pending Tasks Card */}
          <Card className="bg-card border border-border rounded-xl p-6 hover:shadow-lg hover-lift transition-all duration-300 cursor-pointer fade-in-up" style={{ animationDelay: '0.4s' }}>
            <CardHeader className="flex flex-row items-center justify-between pb-4">
              <div>
                <CardTitle className="text-sm font-medium text-muted-foreground">Pending</CardTitle>
                <p className="text-2xl sm:text-3xl font-bold mt-2 text-yellow-600 dark:text-yellow-400">{stats.pending_tasks}</p>
              </div>
              <div className="p-3 rounded-full bg-yellow-100 dark:bg-yellow-900/50">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6 text-yellow-600 dark:text-yellow-400" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <circle cx="12" cy="12" r="10" />
                  <path d="M12 8v4" />
                  <path d="M12 16h.01" />
                </svg>
              </div>
            </CardHeader>
            <CardContent>
              <p className="text-xs text-muted-foreground">Tasks awaiting completion</p>
            </CardContent>
          </Card>

          {/* Completion Rate Card */}
          <Card className="bg-card border border-border rounded-xl p-6 hover:shadow-lg hover-lift transition-all duration-300 cursor-pointer fade-in-up" style={{ animationDelay: '0.5s' }}>
            <CardHeader className="flex flex-row items-center justify-between pb-4">
              <div>
                <CardTitle className="text-sm font-medium text-muted-foreground">Completion Rate</CardTitle>
                <p className="text-2xl sm:text-3xl font-bold mt-2 text-purple-600 dark:text-purple-400">
                  {stats.total_tasks > 0 ? Math.round((stats.completed_tasks / stats.total_tasks) * 100) : 0}%
                </p>
              </div>
              <div className="p-3 rounded-full bg-purple-100 dark:bg-purple-900/50">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6 text-purple-600 dark:text-purple-400" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" />
                  <polyline points="22 4 12 14.01 9 11.01" />
                </svg>
              </div>
            </CardHeader>
            <CardContent>
              <p className="text-xs text-muted-foreground">Overall completion percentage</p>
            </CardContent>
          </Card>
        </div>

        {/* Search & Filter Section */}
        <div className="mb-8 sm:mb-12 fade-in-up" style={{ animationDelay: '0.6s' }}>
          <Card className="bg-card border border-border rounded-xl p-6">
            <CardHeader>
              <CardTitle className="text-lg font-semibold">Task Search & Filter</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="flex flex-col sm:flex-row gap-4">
                <input
                  type="text"
                  placeholder="Search tasks..."
                  value={searchQuery}
                  onChange={(e) => setSearchQuery(e.target.value)}
                  className="flex-1 px-4 py-2 border border-border rounded-lg bg-background text-foreground focus:ring-2 focus:ring-blue-500 focus:border-blue-500 transition-all duration-300"
                />
                <select
                  value={filterStatus}
                  onChange={(e) => setFilterStatus(e.target.value)}
                  className="px-4 py-2 border border-border rounded-lg bg-background text-foreground focus:ring-2 focus:ring-blue-500 focus:border-blue-500 transition-all duration-300"
                >
                  <option value="">All Status</option>
                  <option value="completed">Completed</option>
                  <option value="pending">Pending</option>
                </select>
                <select
                  value={filterPriority}
                  onChange={(e) => setFilterPriority(e.target.value)}
                  className="px-4 py-2 border border-border rounded-lg bg-background text-foreground focus:ring-2 focus:ring-blue-500 focus:border-blue-500 transition-all duration-300"
                >
                  <option value="">All Priorities</option>
                  <option value="high">High</option>
                  <option value="medium">Medium</option>
                  <option value="low">Low</option>
                </select>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Tasks List Panel */}
        <div className="mb-8 sm:mb-12 fade-in-up" style={{ animationDelay: '0.7s' }}>
          <Card className="bg-card border border-border rounded-xl p-6">
            <CardHeader>
              <CardTitle className="text-lg font-semibold">Your Tasks</CardTitle>
            </CardHeader>
            <CardContent>
              {tasksLoading ? (
                <div className="space-y-4">
                  {[1, 2, 3].map((item) => (
                    <div key={item} className="flex items-center justify-between p-3 border border-border rounded-lg animate-pulse">
                      <div className="flex items-center">
                        <div className="h-3 w-3 rounded-full bg-gray-300 dark:bg-gray-600 mr-3"></div>
                        <div className="h-4 bg-gray-200 dark:bg-gray-700 rounded w-32"></div>
                      </div>
                      <div className="h-4 bg-gray-200 dark:bg-gray-700 rounded w-16"></div>
                    </div>
                  ))}
                </div>
              ) : tasks.length === 0 ? (
                <div className="text-center py-8 text-muted-foreground fade-in-up">
                  {searchQuery || filterStatus || filterPriority
                    ? "No tasks match your current filters."
                    : "No tasks found. Create your first task on the Tasks page!"}
                </div>
              ) : (
                <div className="space-y-4">
                  {tasks.map((task, index) => (
                    <div
                      key={task.id}
                      className={`flex items-center justify-between p-3 border border-border rounded-lg transition-all duration-300 hover:shadow-md hover:scale-[1.02] ${
                        task.status === 'completed' ? 'bg-green-50/30 dark:bg-green-900/20' : 'bg-background hover:bg-gray-50/50 dark:hover:bg-gray-800/50'
                      } fade-in-up`}
                      style={{ animationDelay: `${0.8 + index * 0.1}s` }}
                    >
                      <div className="flex items-center">
                        <div className={`h-3 w-3 rounded-full mr-3 ${
                          task.status === 'completed'
                            ? 'bg-green-500'
                            : task.priority === 'high'
                              ? 'bg-red-500'
                              : task.priority === 'medium'
                                ? 'bg-yellow-500'
                                : 'bg-blue-500'
                        }`}></div>
                        <span className={task.status === 'completed' ? 'line-through text-muted-foreground' : ''}>
                          {task.title}
                        </span>
                      </div>
                      <span className={`text-sm ${
                        task.priority === 'high' ? 'text-red-600 dark:text-red-400' :
                        task.priority === 'medium' ? 'text-yellow-600 dark:text-yellow-400' :
                        'text-blue-600 dark:text-blue-400'
                      }`}>
                        {task.priority.charAt(0).toUpperCase() + task.priority.slice(1)}
                      </span>
                    </div>
                  ))}
                </div>
              )}
            </CardContent>
          </Card>
        </div>

        {/* Charts Section */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 sm:gap-8">
          {/* Task Types Chart */}
          <Card className="bg-card border border-border rounded-xl p-6">
            <CardHeader>
              <CardTitle className="text-base sm:text-lg">Task Distribution by Type</CardTitle>
            </CardHeader>
            <CardContent>
              <ResponsiveContainer width="100%" height={300}>
                <BarChart data={typeChartData}>
                  <CartesianGrid strokeDasharray="3 3" strokeOpacity={0.3} />
                  <XAxis dataKey="name" />
                  <YAxis />
                  <Tooltip
                    contentStyle={{
                      backgroundColor: 'hsl(var(--background))',
                      border: '1px solid hsl(var(--border))',
                      borderRadius: '0.5rem'
                    }}
                  />
                  <Bar dataKey="value" fill="hsl(var(--primary))" radius={[4, 4, 0, 0]} />
                </BarChart>
              </ResponsiveContainer>
            </CardContent>
          </Card>

          {/* Task Priorities Chart */}
          <Card className="bg-card border border-border rounded-xl p-6">
            <CardHeader>
              <CardTitle className="text-base sm:text-lg">Task Distribution by Priority</CardTitle>
            </CardHeader>
            <CardContent>
              <ResponsiveContainer width="100%" height={300}>
                <PieChart>
                  <Pie
                    data={priorityChartData}
                    cx="50%"
                    cy="50%"
                    labelLine={false}
                    outerRadius={80}
                    fill="#8884d8"
                    dataKey="value"
                    label={({ name, percent }) => `${name}: ${percent ? (percent * 100).toFixed(0) : '0'}%`}
                  >
                    {priorityChartData.map((entry, index) => (
                      <Cell key={`cell-${index}`} fill={COLORS[index % COLORS.length]} />
                    ))}
                  </Pie>
                  <Tooltip
                    contentStyle={{
                      backgroundColor: 'hsl(var(--background))',
                      border: '1px solid hsl(var(--border))',
                      borderRadius: '0.5rem'
                    }}
                  />
                </PieChart>
              </ResponsiveContainer>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}