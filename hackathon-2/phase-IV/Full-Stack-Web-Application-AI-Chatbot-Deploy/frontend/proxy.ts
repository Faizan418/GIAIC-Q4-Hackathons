import { NextResponse } from 'next/server';
import type { NextRequest } from 'next/server';

// Proxy for route protection
export async function proxy(request: NextRequest) {
  // Check if this is an API task route that requires authentication
  const isApiTaskRoute = request.nextUrl.pathname.includes('/api/tasks');

  if (isApiTaskRoute) {
    // For our backend authentication system, we'll check for our auth token
    // This can be done by checking if the user is authenticated via our auth API
    // Since this is a proxy, we'll allow the request to proceed and let the backend handle authentication
    // Or we can check if the user has our auth token stored
    const authToken = request.cookies.get('authToken')?.value || null;

    // Alternatively, we can make a lightweight request to our backend to verify the token
    // For now, we'll let the backend handle the authentication
  }

  // Allow the request to continue
  return NextResponse.next();
}

// Define which paths the middleware should run on
export const config = {
  matcher: ['/api/tasks/:path*'], // Only protect API task routes, allow auth routes
};