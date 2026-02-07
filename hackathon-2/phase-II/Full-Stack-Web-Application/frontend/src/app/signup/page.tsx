'use client';

import { useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { Button } from '@/components/ui/button';
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from '@/components/ui/card';
import { ThemeToggle } from '@/components/ui/theme-toggle';
import Link from 'next/link';

export default function SignUpPage() {
  const router = useRouter();

  // Redirect to dashboard since authentication is no longer required
  useEffect(() => {
    const timer = setTimeout(() => {
      router.push('/dashboard');
    }, 3000); // Redirect after 3 seconds to show message

    return () => clearTimeout(timer);
  }, [router]);

  return (
    <div className="min-h-screen flex items-center justify-center bg-gradient-to-br from-blue-50 to-indigo-100 dark:from-gray-900 dark:to-gray-800 p-4">
      <Card className="w-full max-w-md bg-white/80 dark:bg-gray-800/80 backdrop-blur-sm border border-glass-border shadow-glass">
        <CardHeader className="space-y-1">
          <div className="flex justify-between items-center">
            <CardTitle className="text-2xl font-bold bg-clip-text text-transparent bg-gradient-to-r from-blue-600 to-purple-600">
              TASKAPP
            </CardTitle>
            <ThemeToggle />
          </div>
          <CardTitle className="text-2xl">Public Access</CardTitle>
          <CardDescription>
            Authentication is no longer required. Redirecting to dashboard...
          </CardDescription>
        </CardHeader>
        <CardContent>
          <p className="text-center text-gray-600 dark:text-gray-300 mb-4">
            TASKAPP is now publicly accessible. No registration required!
          </p>
          <div className="text-center">
            <Button asChild>
              <Link href="/dashboard">Go to Dashboard</Link>
            </Button>
          </div>
        </CardContent>
        <CardFooter className="flex flex-col space-y-4">
          <div className="text-center text-sm text-gray-600 dark:text-gray-400">
            Want to continue?{' '}
            <Link href="/dashboard" className="font-medium text-blue-600 hover:underline dark:text-blue-400">
              Visit Dashboard
            </Link>
          </div>
        </CardFooter>
      </Card>
    </div>
  );
}