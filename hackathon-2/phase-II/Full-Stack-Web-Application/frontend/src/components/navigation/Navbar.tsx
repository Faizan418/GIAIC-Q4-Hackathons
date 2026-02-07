'use client';

import React from 'react';
import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { Moon, Sun, Menu, X } from 'lucide-react';
import { useTheme } from 'next-themes';
import { Button } from '@/components/ui/button';
import { Sheet, SheetContent, SheetTrigger, SheetHeader, SheetTitle } from '@/components/ui/sheet';

const Navbar = () => {
  const pathname = usePathname();
  const { theme, setTheme } = useTheme();
  const [mounted, setMounted] = React.useState(false);

  React.useEffect(() => {
    setMounted(true);
  }, []);

  const navigationLinks = [
    { name: 'Dashboard', href: '/dashboard' as const },
    { name: 'Tasks', href: '/tasks' as const },
    { name: 'Sign In', href: '/signin' as const },
    { name: 'Sign Up', href: '/signup' as const },
  ];

  const isActive = (href: string) => pathname === href;

  const toggleTheme = () => {
    setTheme(theme === 'dark' ? 'light' : 'dark');
  };

  return (
    <header className="fixed top-0 left-0 right-0 z-50 border-b bg-white/80 dark:bg-gray-900/80 backdrop-blur-sm border-gray-200 dark:border-gray-700">
      <div className="container mx-auto px-4 max-w-screen-xl">
        <div className="flex h-[72px] items-center justify-between">
          {/* Logo */}
          <div className="flex items-center">
            <Link
              href="/"
              className="text-2xl font-bold text-blue-600 dark:text-blue-400"
              aria-label="TASKAPP Home"
            >
              TASKAPP
            </Link>
          </div>

          {/* Desktop Navigation */}
          <nav className="hidden md:flex items-center space-x-8" aria-label="Main navigation">
            {navigationLinks.map((link) => (
              <Link
                key={link.href}
                href={link.href as any}
                className={`text-sm font-medium transition-all duration-300 hover:text-blue-600 dark:hover:text-blue-400 hover:translate-x-1 ${
                  isActive(link.href)
                    ? 'text-blue-600 dark:text-blue-400'
                    : 'text-gray-600 dark:text-gray-300'
                }`}
                aria-current={isActive(link.href) ? "page" : undefined}
              >
                {link.name}
              </Link>
            ))}
          </nav>

          {/* Theme Toggle and Mobile Menu */}
          <div className="flex items-center space-x-4">
            <Button
              variant="ghost"
              size="icon"
              onClick={toggleTheme}
              aria-label="Toggle theme"
              className="h-8 w-8"
            >
              {mounted ? (
                theme === 'dark' ? (
                  <Sun className="h-5 w-5 text-yellow-500" aria-hidden="true" />
                ) : (
                  <Moon className="h-5 w-5 text-white" aria-hidden="true" />
                )
              ) : (
                // Render a placeholder during hydration to avoid mismatch
                <Sun className="h-5 w-5 text-yellow-500 opacity-0" aria-hidden="true" />
              )}
            </Button>

            {/* Mobile Menu */}
            <Sheet>
              <SheetTrigger asChild>
                <Button
                  variant="ghost"
                  size="icon"
                  className="md:hidden h-8 w-8"
                  aria-label="Open menu"
                >
                  <Menu className="h-5 w-5" aria-hidden="true" />
                </Button>
              </SheetTrigger>
              <SheetContent side="right" className="w-[300px] bg-white dark:bg-gray-900">
                <SheetHeader>
                  <SheetTitle>Navigation Menu</SheetTitle>
                </SheetHeader>
                <div className="flex flex-col space-y-6 mt-6">
                  {navigationLinks.map((link) => (
                    <Link
                      key={link.href}
                      href={link.href as any}
                      className={`text-lg font-medium ${
                        isActive(link.href)
                          ? 'text-blue-600 dark:text-blue-400'
                          : 'text-gray-600 dark:text-gray-300'
                      }`}
                      aria-current={isActive(link.href) ? "page" : undefined}
                    >
                      {link.name}
                    </Link>
                  ))}
                  <Button
                    variant="outline"
                    onClick={toggleTheme}
                    className="mt-4 w-full justify-center"
                    aria-label={`Switch to ${mounted && theme === 'dark' ? 'light' : 'dark'} theme`}
                  >
                    {mounted && theme === 'dark' ? 'Switch to Light' : 'Switch to Dark'}
                  </Button>
                </div>
              </SheetContent>
            </Sheet>
          </div>
        </div>
      </div>
    </header>
  );
};

export default Navbar;