import { auth } from '@/lib/better-auth';

// Create the Better Auth API routes
const handler = auth.handler;

export { handler as GET, handler as POST };