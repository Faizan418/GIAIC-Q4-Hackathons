// Type definitions for chat components

export interface ChatMessage {
  id: string;
  sender: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  status?: 'pending' | 'sent' | 'delivered' | 'error';
  parentId?: string;
}

export interface ConversationSession {
  sessionId: string;
  messages: ChatMessage[];
  createdAt: Date;
  lastActiveAt: Date;
  isActive: boolean;
}

export interface TextSelection {
  id: string;
  content: string;
  context: string;
  position: { start: number; end: number };
  timestamp: Date;
  sourceUrl: string;
}

export interface QueryRequest {
  query: string;
  conversation_id?: string;
  context?: {
    page_url: string;
    previous_messages?: {
      role: 'user' | 'assistant';
      content: string;
    }[];
  };
}

export interface QueryResponse {
  response_id: string;
  answer: string;
  sources: Array<{
    title: string;
    url: string;
    snippet: string;
  }>;
  conversation_id: string;
}

export interface TextSelectionQueryRequest {
  selected_text: string;
  context: {
    page_url: string;
    surrounding_text: string;
    conversation_id?: string;
  };
}

export interface TextSelectionQueryResponse {
  response_id: string;
  explanation: string;
  related_concepts: string[];
  sources: Array<{
    title: string;
    url: string;
    snippet: string;
  }>;
  conversation_id: string;
}

// Auth-related types
export interface UserPublic {
  id: string;
  email: string;
  created_at: string; // ISO date string
  updated_at: string; // ISO date string
  email_verified: boolean;
  profile_complete: boolean;
}

export interface UserProfilePublic {
  id: string;
  user_id: string;
  software_level: string; // "BEGINNER" | "INTERMEDIATE" | "ADVANCED"
  known_languages: string[]; // Array of language strings
  hardware_experience: string; // "NONE" | "BASIC" | "INTERMEDIATE" | "ADVANCED"
  learning_track: string; // "SOFTWARE_ONLY" | "HARDWARE_ONLY" | "FULL_ROBOTICS"
  boards_used: string[]; // Array of board strings
  created_at: string; // ISO date string
  updated_at: string; // ISO date string
}

export interface AuthResponse {
  user: UserPublic;
  access_token: string;
  token_type: string;
}

export interface UserProfileCreate {
  software_level: string;
  known_languages: string[];
  hardware_experience: string;
  learning_track: string;
  boards_used?: string[];
}

export interface UserProfileUpdate {
  software_level?: string;
  known_languages?: string[];
  hardware_experience?: string;
  learning_track?: string;
  boards_used?: string[];
}
