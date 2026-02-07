"use client";

import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import Link from "next/link";
import { CheckCircle, Calendar, Flag, Tag, Zap, Clock, Users, BarChart3 } from "lucide-react";

export default function HomePage() {

  return (
    <div className="min-h-screen bg-background">
      {/* Hero Section */}
      <section className="min-h-screen flex flex-col md:flex-row items-center justify-between pt-24 pb-16 px-4 sm:px-6 lg:px-8">
        <div className="md:w-1/2 mb-12 md:mb-0 text-center md:text-left px-4 max-w-2xl fade-in-up" style={{ animationDelay: '0.1s' }}>
          <h1 className="text-4xl sm:text-5xl md:text-6xl font-bold mb-6 leading-tight fade-in-up" style={{ animationDelay: '0.1s' }}>
            Master Your Day with <span className="text-blue-600 dark:text-blue-400">Intelligent Task Automation</span>
          </h1>
          <p className="text-lg md:text-xl text-gray-600 dark:text-gray-300 mb-8 max-w-lg fade-in-up" style={{ animationDelay: '0.2s' }}>
            Create, automate, and track tasks effortlessly. Built for professionals who value clarity and time.
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center md:justify-start fade-in-up" style={{ animationDelay: '0.3s' }}>
            <Link href="/dashboard">
              <Button size="lg" className="bg-blue-600 hover:bg-blue-700 text-white text-base sm:text-lg px-6 py-3 sm:px-8 sm:py-4 rounded-lg btn-hover-scale">
                Get Started — Free
              </Button>
            </Link>
            <Link href="#features">
              <Button size="lg" variant="outline" className="text-base sm:text-lg px-6 py-3 sm:px-8 sm:py-4 rounded-lg border-gray-300 dark:border-gray-600 btn-hover-scale">
                See how it works
              </Button>
            </Link>
          </div>
        </div>

        {/* Right Column - Animated video demo of CRUD operations */}
        <div className="md:w-1/2 flex justify-center px-4 fade-in-up" style={{ animationDelay: '0.4s' }}>
          <div className="bg-gray-100 dark:bg-gray-800 rounded-2xl p-6 w-full max-w-lg border border-gray-200 dark:border-gray-700 overflow-hidden">
            <div className="aspect-video bg-gradient-to-br from-blue-50 to-indigo-100 dark:from-gray-800 dark:to-gray-900 rounded-lg flex items-center justify-center relative overflow-hidden">
              {/* Animated CRUD Operation Simulation */}
              <div className="absolute inset-0 flex flex-col items-center justify-center p-4">
                <div className="space-y-3 w-full max-w-xs">
                  {/* Create Task Animation */}
                  <div className="flex items-center p-3 bg-white dark:bg-gray-700 rounded-lg border border-gray-200 dark:border-gray-600 slide-in-left">
                    <div className="h-3 w-3 rounded-full bg-red-500 mr-3 animate-pulse"></div>
                    <span className="text-sm font-medium">Creating Task...</span>
                  </div>

                  {/* Update Task Animation */}
                  <div className="flex items-center p-3 bg-white dark:bg-gray-700 rounded-lg border border-gray-200 dark:border-gray-600 slide-in-left" style={{ animationDelay: '1s' }}>
                    <div className="h-3 w-3 rounded-full bg-yellow-500 mr-3 animate-pulse"></div>
                    <span className="text-sm font-medium">Updating Task...</span>
                  </div>

                  {/* Complete Task Animation */}
                  <div className="flex items-center p-3 bg-white dark:bg-gray-700 rounded-lg border border-gray-200 dark:border-gray-600 slide-in-left" style={{ animationDelay: '2s' }}>
                    <div className="h-3 w-3 rounded-full bg-green-500 mr-3 animate-pulse"></div>
                    <span className="text-sm font-medium">Completing Task...</span>
                  </div>

                  {/* Delete Task Animation */}
                  <div className="flex items-center p-3 bg-white dark:bg-gray-700 rounded-lg border border-gray-200 dark:border-gray-600 slide-in-left" style={{ animationDelay: '3s' }}>
                    <div className="h-3 w-3 rounded-full bg-red-500 mr-3 animate-pulse"></div>
                    <span className="text-sm font-medium">Deleting Task...</span>
                  </div>
                </div>

                <div className="mt-6 text-center">
                  <p className="text-sm font-medium text-gray-700 dark:text-gray-300 animate-pulse">Automated Task Operations</p>
                  <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">Create • Update • Complete • Delete</p>
                </div>
              </div>

              {/* Overlay for video-like appearance */}
              <div className="absolute inset-0 bg-black/10 dark:bg-black/20 rounded-lg pointer-events-none"></div>
            </div>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section id="features" className="py-20 px-4 sm:px-6 lg:px-8 fade-in-up" style={{ animationDelay: '0.5s' }}>
        <div className="max-w-7xl mx-auto">
          <h2 className="text-3xl sm:text-4xl font-bold text-center mb-16 fade-in-up" style={{ animationDelay: '0.5s' }}>Powerful Features</h2>
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-6 sm:gap-8">
            <Card className="bg-card border border-gray-200 dark:border-gray-700 rounded-xl p-6 hover:shadow-lg hover-lift transition-all duration-300 cursor-pointer fade-in-up" style={{ animationDelay: '0.6s' }}>
              <div className="p-3 rounded-full bg-blue-100 dark:bg-blue-900/50 mb-4 inline-block">
                <CheckCircle className="h-6 w-6 text-blue-600 dark:text-blue-400" />
              </div>
              <CardHeader className="p-0">
                <CardTitle className="text-lg">Smart Task Management</CardTitle>
                <CardDescription className="text-sm">Organize and prioritize your tasks with ease</CardDescription>
              </CardHeader>
              <CardContent className="p-0 pt-2">
                Create, update, and manage tasks with intuitive controls and smart defaults.
              </CardContent>
            </Card>

            <Card className="bg-card border border-gray-200 dark:border-gray-700 rounded-xl p-6 hover:shadow-lg hover-lift transition-all duration-300 cursor-pointer fade-in-up" style={{ animationDelay: '0.7s' }}>
              <div className="p-3 rounded-full bg-blue-100 dark:bg-blue-900/50 mb-4 inline-block">
                <Calendar className="h-6 w-6 text-blue-600 dark:text-blue-400" />
              </div>
              <CardHeader className="p-0">
                <CardTitle className="text-lg">Due Dates & Reminders</CardTitle>
                <CardDescription className="text-sm">Never miss a deadline again</CardDescription>
              </CardHeader>
              <CardContent className="p-0 pt-2">
                Set due dates and receive timely reminders to stay on track with your goals.
              </CardContent>
            </Card>

            <Card className="bg-card border border-gray-200 dark:border-gray-700 rounded-xl p-6 hover:shadow-lg hover-lift transition-all duration-300 cursor-pointer fade-in-up" style={{ animationDelay: '0.8s' }}>
              <div className="p-3 rounded-full bg-blue-100 dark:bg-blue-900/50 mb-4 inline-block">
                <Flag className="h-6 w-6 text-blue-600 dark:text-blue-400" />
              </div>
              <CardHeader className="p-0">
                <CardTitle className="text-lg">Priority Levels</CardTitle>
                <CardDescription className="text-sm">Focus on what matters most</CardDescription>
              </CardHeader>
              <CardContent className="p-0 pt-2">
                Assign priority levels to tasks and tackle the most important items first.
              </CardContent>
            </Card>

            <Card className="bg-card border border-gray-200 dark:border-gray-700 rounded-xl p-6 hover:shadow-lg hover-lift transition-all duration-300 cursor-pointer fade-in-up" style={{ animationDelay: '0.9s' }}>
              <div className="p-3 rounded-full bg-blue-100 dark:bg-blue-900/50 mb-4 inline-block">
                <Tag className="h-6 w-6 text-blue-600 dark:text-blue-400" />
              </div>
              <CardHeader className="p-0">
                <CardTitle className="text-lg">Tag Organization</CardTitle>
                <CardDescription className="text-sm">Categorize and filter your tasks</CardDescription>
              </CardHeader>
              <CardContent className="p-0 pt-2">
                Use tags to organize tasks by project, context, or any custom category.
              </CardContent>
            </Card>
          </div>
        </div>
      </section>

      {/* How It Works Section */}
      <section className="py-20 px-4 sm:px-6 lg:px-8">
        <div className="max-w-7xl mx-auto">
          <h2 className="text-3xl sm:text-4xl font-bold text-center mb-16">How It Works</h2>
          <div className="grid grid-cols-1 md:grid-cols-4 gap-8">
            <div className="text-center">
              <div className="mx-auto flex items-center justify-center h-16 w-16 rounded-full bg-blue-100 dark:bg-blue-900/50 mb-4">
                <span className="text-2xl font-bold text-blue-600 dark:text-blue-400">1</span>
              </div>
              <h3 className="text-xl font-semibold mb-2">Create</h3>
              <p className="text-gray-600 dark:text-gray-300">Add tasks with details, due dates, and priorities</p>
            </div>
            <div className="text-center">
              <div className="mx-auto flex items-center justify-center h-16 w-16 rounded-full bg-blue-100 dark:bg-blue-900/50 mb-4">
                <span className="text-2xl font-bold text-blue-600 dark:text-blue-400">2</span>
              </div>
              <h3 className="text-xl font-semibold mb-2">Organize</h3>
              <p className="text-gray-600 dark:text-gray-300">Categorize and prioritize your tasks</p>
            </div>
            <div className="text-center">
              <div className="mx-auto flex items-center justify-center h-16 w-16 rounded-full bg-blue-100 dark:bg-blue-900/50 mb-4">
                <span className="text-2xl font-bold text-blue-600 dark:text-blue-400">3</span>
              </div>
              <h3 className="text-xl font-semibold mb-2">Track</h3>
              <p className="text-gray-600 dark:text-gray-300">Monitor progress and get insights</p>
            </div>
            <div className="text-center">
              <div className="mx-auto flex items-center justify-center h-16 w-16 rounded-full bg-blue-100 dark:bg-blue-900/50 mb-4">
                <span className="text-2xl font-bold text-blue-600 dark:text-blue-400">4</span>
              </div>
              <h3 className="text-xl font-semibold mb-2">Achieve</h3>
              <p className="text-gray-600 dark:text-gray-300">Complete tasks and reach your goals</p>
            </div>
          </div>
        </div>
      </section>

      {/* Dashboard Preview Section */}
      <section className="py-20 px-4 sm:px-6 lg:px-8">
        <div className="max-w-7xl mx-auto">
          <h2 className="text-3xl sm:text-4xl font-bold text-center mb-16">Dashboard Preview</h2>
          <div className="bg-card border border-gray-200 dark:border-gray-700 rounded-2xl p-8 hover:scale-105 transition-transform duration-300">
            <div className="aspect-video bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center">
              <div className="text-center">
                <BarChart3 className="h-12 w-12 text-blue-600 dark:text-blue-400 mx-auto mb-4" />
                <p className="text-lg font-medium">Interactive Dashboard Preview</p>
                <p className="text-gray-500 dark:text-gray-400">Track your productivity metrics</p>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="py-20 px-4 sm:px-6 lg:px-8">
        <div className="max-w-3xl mx-auto text-center">
          <h2 className="text-3xl sm:text-4xl font-bold mb-6">Ready to Transform Your Productivity?</h2>
          <p className="text-lg text-gray-600 dark:text-gray-300 mb-10 max-w-2xl mx-auto">
            Join thousands of users who have revolutionized their task management with TASKAPP.
          </p>
          <Link href="/dashboard">
            <Button size="lg" className="bg-gradient-to-r from-blue-600 to-blue-700 hover:from-blue-700 hover:to-blue-800 text-white text-lg px-8 py-4 rounded-lg animate-pulse">
              Start Free Trial
            </Button>
          </Link>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t border-gray-200 dark:border-gray-700 py-8">
        <div className="max-w-7xl mx-auto px-4 text-center text-gray-600 dark:text-gray-400">
          <p>© {new Date().getFullYear()} TASKAPP. All rights reserved.</p>
        </div>
      </footer>
    </div>
  );
}