'use client';

import * as React from 'react';
import { Button } from '@/components/ui/button';
import { Card } from '@/components/ui/card';
import { ChevronLeft, ChevronRight, Plus } from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';

interface SidebarProps {
  isOpen: boolean;
  onToggle: () => void;
  title: string;
  children: React.ReactNode;
}

export function Sidebar({ isOpen, onToggle, title, children }: SidebarProps) {
  return (
    <div className="flex">
      <motion.div
        initial={false}
        animate={{
          width: isOpen ? '300px' : '60px',
        }}
        transition={{ duration: 0.3, ease: 'easeInOut' }}
        className="flex flex-col"
      >
        <Card className="bg-white/80 dark:bg-gray-800/80 backdrop-blur-sm border border-glass-border shadow-glass h-full">
          <div className="p-4 border-b border-gray-200 dark:border-gray-700 flex items-center justify-between">
            <AnimatePresence>
              {isOpen && (
                <motion.h3
                  initial={{ opacity: 0, x: -10 }}
                  animate={{ opacity: 1, x: 0 }}
                  exit={{ opacity: 0, x: -10 }}
                  className="font-semibold text-lg"
                >
                  {title}
                </motion.h3>
              )}
            </AnimatePresence>
            <Button
              variant="ghost"
              size="sm"
              onClick={onToggle}
              className="shrink-0"
            >
              {isOpen ? <ChevronLeft className="h-4 w-4" /> : <ChevronRight className="h-4 w-4" />}
              <span className="sr-only">
                {isOpen ? 'Collapse sidebar' : 'Expand sidebar'}
              </span>
            </Button>
          </div>
          <div className="p-4 flex-1 overflow-y-auto">
            <AnimatePresence>
              {isOpen && (
                <motion.div
                  initial={{ opacity: 0 }}
                  animate={{ opacity: 1 }}
                  exit={{ opacity: 0 }}
                  transition={{ delay: 0.1 }}
                >
                  {children}
                </motion.div>
              )}
            </AnimatePresence>
          </div>
        </Card>
      </motion.div>

      {!isOpen && (
        <div className="absolute left-4 z-10 mt-2">
          <Button
            variant="default"
            size="sm"
            onClick={onToggle}
            className="rounded-full h-8 w-8 p-0"
          >
            <Plus className="h-4 w-4" />
            <span className="sr-only">Expand sidebar</span>
          </Button>
        </div>
      )}
    </div>
  );
}