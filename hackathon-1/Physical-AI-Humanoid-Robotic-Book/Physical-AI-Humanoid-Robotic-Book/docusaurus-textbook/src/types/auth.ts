// Auth-related TypeScript types

export interface User {
  id: string;
  email: string;
  createdAt: Date;
  updatedAt: Date;
  emailVerified: boolean;
  metadata?: UserMetadata;
}

export interface UserMetadata {
  softwareBackground?: string;
  hardwareExperience?: string;
  learningTrack?: 'SOFTWARE_ONLY' | 'HARDWARE_ONLY' | 'FULL_ROBOTICS';
  skillLevel?: 'BEGINNER' | 'INTERMEDIATE' | 'ADVANCED';
}

export interface AuthSession {
  sessionId: string;
  userId: string;
  expiresAt: Date;
  createdAt: Date;
  ipAddress?: string;
  userAgent?: string;
}

export interface ProtectedRoute {
  path: string;
  name: string;
  description: string;
  redirectPath: string;
}

export interface DashboardContent {
  userId: string;
  welcomeMessage: string;
  backgroundDisplay: string;
  recommendedModules: string[];
  lastAccessed: Date;
}

export interface FormSubmission {
  userId: string;
  formData: Record<string, any>;
  status: 'PENDING' | 'COMPLETED' | 'FAILED';
  submittedAt: Date;
  processedAt?: Date;
}

// Better Auth specific types
export interface AuthConfig {
  secret: string;
  databaseUrl: string;
  sessionExpires: number;
}

// API request/response types
export interface SignupRequest {
  email: string;
  password: string;
  softwareBackground: string;
  hardwareExperience: string;
  learningTrack?: 'SOFTWARE_ONLY' | 'HARDWARE_ONLY' | 'FULL_ROBOTICS';
  skillLevel?: 'BEGINNER' | 'INTERMEDIATE' | 'ADVANCED';
}

export interface SigninRequest {
  email: string;
  password: string;
}

export interface AuthResponse {
  user?: User;
  session?: AuthSession;
  error?: string;
}

export interface SessionResponse {
  user: User;
  session: AuthSession;
}
