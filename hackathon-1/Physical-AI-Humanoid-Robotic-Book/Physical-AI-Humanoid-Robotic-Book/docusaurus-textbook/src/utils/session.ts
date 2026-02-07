// Session management utility for chat conversations

import { ConversationSession, ChatMessage } from '../types/chat';

const SESSION_STORAGE_KEY = 'chat-session';
const MAX_MESSAGES = 100;

class SessionManager {
  private session: ConversationSession | null = null;

  constructor() {
    // Only load session on client side
    if (typeof window !== 'undefined') {
      this.loadSession();
    }
  }

  private generateSessionId(): string {
    return `sess_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateMessageId(): string {
    return `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  createSession(): ConversationSession {
    this.session = {
      sessionId: this.generateSessionId(),
      messages: [],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    };
    this.saveSession();
    return this.session;
  }

  getSession(): ConversationSession {
    if (!this.session) {
      return this.createSession();
    }
    return this.session;
  }

  addMessage(message: Omit<ChatMessage, 'id' | 'timestamp'>): ChatMessage {
    if (!this.session) {
      this.createSession();
    }

    const newMessage: ChatMessage = {
      ...message,
      id: this.generateMessageId(),
      timestamp: new Date(),
    };

    if (this.session) {
      // Add the new message
      this.session.messages.push(newMessage);

      // Limit the number of messages to prevent memory issues
      if (this.session.messages.length > MAX_MESSAGES) {
        this.session.messages = this.session.messages.slice(-MAX_MESSAGES);
      }

      this.session.lastActiveAt = new Date();
      this.saveSession();
    }

    return newMessage;
  }

  getMessages(): ChatMessage[] {
    return this.session?.messages || [];
  }

  clearMessages(): void {
    if (this.session) {
      this.session.messages = [];
      this.session.lastActiveAt = new Date();
      this.saveSession();
    }
  }

  updateMessageStatus(messageId: string, status: ChatMessage['status']): void {
    if (this.session) {
      const message = this.session.messages.find(msg => msg.id === messageId);
      if (message) {
        message.status = status;
        this.session.lastActiveAt = new Date();
        this.saveSession();
      }
    }
  }

  private saveSession(): void {
    // Only save to sessionStorage on client side
    if (typeof window !== 'undefined' && this.session) {
      try {
        const sessionToSave = {
          ...this.session,
          createdAt: this.session.createdAt.toISOString(),
          lastActiveAt: this.session.lastActiveAt.toISOString(),
        };
        sessionStorage.setItem(SESSION_STORAGE_KEY, JSON.stringify(sessionToSave));
      } catch (error) {
        console.error('Error saving session to sessionStorage:', error);
      }
    }
  }

  private loadSession(): void {
    // Only load from sessionStorage on client side
    if (typeof window === 'undefined') {
      return;
    }

    try {
      const sessionData = sessionStorage.getItem(SESSION_STORAGE_KEY);
      if (sessionData) {
        const parsed = JSON.parse(sessionData);
        this.session = {
          ...parsed,
          createdAt: new Date(parsed.createdAt),
          lastActiveAt: new Date(parsed.lastActiveAt),
          messages: parsed.messages.map((msg: any) => ({
            ...msg,
            timestamp: new Date(msg.timestamp),
          })),
        };
      }
    } catch (error) {
      console.error('Error loading session from sessionStorage:', error);
      // Create a new session if loading fails
      this.createSession();
    }
  }

  endSession(): void {
    if (this.session) {
      this.session.isActive = false;
      this.session.lastActiveAt = new Date();
      this.saveSession();
    }
  }

  // Get the current conversation ID for API calls
  getConversationId(): string | undefined {
    return this.session?.sessionId;
  }
}

export default new SessionManager();
