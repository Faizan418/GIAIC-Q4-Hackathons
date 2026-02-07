'use client';

import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { CheckCircle, Clock, Flag, Move3D, Trash2 } from 'lucide-react';

const AnimatedDashboard = () => {
  const [currentStep, setCurrentStep] = useState(0);
  const [isPlaying, setIsPlaying] = useState(true);

  // Animation sequence steps
  const steps = [
    { id: 0, description: "Empty task list", action: "Initial state" },
    { id: 1, description: "User types task", action: "Task appears" },
    { id: 2, description: "Task status changes", action: "Check animation" },
    { id: 3, description: "Drag & drop reorder", action: "Reorder visualization" },
    { id: 4, description: "Quick delete", action: "Fade-out delete" },
    { id: 5, description: "Automation highlight", action: "Automation badge" }
  ];

  // Sample tasks for the animation
  const [tasks, setTasks] = useState<any[]>([]);
  const [showAutomationBadge, setShowAutomationBadge] = useState(false);

  useEffect(() => {
    let interval: NodeJS.Timeout | null = null;

    if (isPlaying) {
      interval = setInterval(() => {
        setCurrentStep(prev => (prev + 1) % steps.length);
      }, 2000); // Change step every 2 seconds
    }

    return () => {
      if (interval) clearInterval(interval);
    };
  }, [isPlaying]);

  useEffect(() => {
    // Execute animation based on current step
    switch(currentStep) {
      case 0: // Empty state
        setTasks([]);
        setShowAutomationBadge(false);
        break;
      case 1: // Add task
        setTimeout(() => {
          setTasks([{
            id: 1,
            title: "Complete project proposal",
            completed: false,
            priority: 'high',
            dueDate: '2026-01-20'
          }]);
        }, 300);
        break;
      case 2: // Mark as complete
        setTimeout(() => {
          setTasks(prev => prev.map(task =>
            task.id === 1 ? {...task, completed: true} : task
          ));
        }, 500);
        break;
      case 3: // Reorder visualization
        // No action needed, just visualize
        break;
      case 4: // Delete task
        setTimeout(() => {
          setTasks([]);
        }, 300);
        break;
      case 5: // Automation badge
        setTimeout(() => {
          setShowAutomationBadge(true);
        }, 500);
        break;
    }
  }, [currentStep]);

  return (
    <div className="relative w-full max-w-2xl h-96 bg-gray-800/30 backdrop-blur-sm border border-glass-border rounded-xl p-6 overflow-hidden">
      {/* Header */}
      <div className="flex justify-between items-center mb-4">
        <h3 className="text-white font-semibold">My Tasks</h3>
        <div className="flex space-x-2">
          <div className="w-3 h-3 bg-red-500 rounded-full"></div>
          <div className="w-3 h-3 bg-yellow-500 rounded-full"></div>
          <div className="w-3 h-3 bg-green-500 rounded-full"></div>
        </div>
      </div>

      {/* Task List */}
      <div className="space-y-3">
        <AnimatePresence>
          {tasks.map((task) => (
            <motion.div
              key={task.id}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              exit={{ opacity: 0, y: -20 }}
              transition={{ duration: 0.3 }}
              className={`flex items-center justify-between p-3 rounded-lg ${
                task.completed
                  ? 'bg-green-500/20 border border-green-500/30'
                  : 'bg-gray-700/50 border border-gray-600'
              }`}
            >
              <div className="flex items-center space-x-3">
                <motion.button
                  whileTap={{ scale: 0.9 }}
                  className={`p-1 rounded-full ${
                    task.completed ? 'text-green-400' : 'text-gray-400'
                  }`}
                >
                  {task.completed ? (
                    <CheckCircle className="h-5 w-5" />
                  ) : (
                    <div className="h-5 w-5 rounded-full border border-gray-400" />
                  )}
                </motion.button>
                <div>
                  <p className={`font-medium ${task.completed ? 'line-through text-gray-400' : 'text-white'}`}>
                    {task.title}
                  </p>
                  <div className="flex items-center space-x-2 mt-1">
                    <span className="text-xs px-2 py-1 bg-blue-500/20 text-blue-300 rounded">
                      {task.priority}
                    </span>
                    <span className="text-xs text-gray-400 flex items-center">
                      <Clock className="h-3 w-3 mr-1" /> {task.dueDate}
                    </span>
                  </div>
                </div>
              </div>
              <motion.button
                whileHover={{ scale: 1.1 }}
                whileTap={{ scale: 0.9 }}
                className="text-gray-400 hover:text-white"
              >
                <Trash2 className="h-4 w-4" />
              </motion.button>
            </motion.div>
          ))}
        </AnimatePresence>

        {/* Empty state */}
        {tasks.length === 0 && currentStep !== 1 && currentStep !== 5 && (
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            className="text-center py-10 text-gray-400"
          >
            <p>No tasks yet</p>
          </motion.div>
        )}

        {/* Automation badge */}
        {showAutomationBadge && (
          <motion.div
            initial={{ scale: 0, opacity: 0 }}
            animate={{ scale: 1, opacity: 1 }}
            className="absolute top-4 right-4 flex items-center bg-blue-500/20 text-blue-300 px-3 py-1 rounded-full text-sm"
          >
            <Flag className="h-4 w-4 mr-1" />
            Automated
          </motion.div>
        )}
      </div>

      {/* Reordering visualization */}
      {(currentStep === 3) && (
        <>
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            className="absolute inset-0 bg-blue-500/10 backdrop-blur-sm"
          />
          <motion.div
            initial={{ x: 0 }}
            animate={{ x: [-10, 10, -10, 0] }}
            transition={{ duration: 1, repeat: Infinity }}
            className="absolute bottom-4 right-4 text-blue-300"
          >
            <Move3D className="h-8 w-8" />
          </motion.div>
        </>
      )}

      {/* Progress indicator */}
      <div className="absolute bottom-4 left-1/2 transform -translate-x-1/2 flex items-center space-x-4">
        <div className="flex space-x-1">
          {steps.map((_, index) => (
            <div
              key={index}
              className={`w-2 h-2 rounded-full ${
                index === currentStep ? 'bg-blue-500' : 'bg-gray-600'
              }`}
            />
          ))}
        </div>

        {/* Play/Pause Controls */}
        <button
          onClick={() => setIsPlaying(!isPlaying)}
          className="ml-2 p-2 rounded-full bg-black/20 hover:bg-black/30 transition-colors"
          aria-label={isPlaying ? "Pause animation" : "Play animation"}
        >
          {isPlaying ? (
            <div className="flex space-x-1">
              <div className="w-1 h-4 bg-white"></div>
              <div className="w-1 h-4 bg-white"></div>
            </div>
          ) : (
            <div className="w-0 h-0 border-t-2 border-b-2 border-l-4 border-transparent border-l-white ml-1"></div>
          )}
        </button>
      </div>
    </div>
  );
};

export default AnimatedDashboard;