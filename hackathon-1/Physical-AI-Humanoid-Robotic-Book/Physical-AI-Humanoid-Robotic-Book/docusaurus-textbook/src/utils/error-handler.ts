// Error handling utility for chat components

export class ChatError extends Error {
  public readonly code: string;
  public readonly details?: any;

  constructor(message: string, code: string = 'UNKNOWN_ERROR', details?: any) {
    super(message);
    this.name = 'ChatError';
    this.code = code;
    this.details = details;
  }
}

export const handleApiError = (error: any): ChatError => {
  if (error instanceof ChatError) {
    return error;
  }

  if (error instanceof TypeError && error.message.includes('fetch')) {
    return new ChatError('Network error: Unable to connect to the server', 'NETWORK_ERROR');
  }

  if (error.response) {
    // Server responded with error status
    const status = error.response.status;
    if (status >= 500) {
      return new ChatError('Server error: The service is temporarily unavailable', 'SERVER_ERROR');
    } else if (status >= 400) {
      return new ChatError(`Request error: ${error.response.data?.error || 'Bad request'}`, 'REQUEST_ERROR');
    }
  }

  // Fallback for other errors
  return new ChatError(error.message || 'An unknown error occurred', 'UNKNOWN_ERROR', error);
};

export const displayErrorMessage = (error: ChatError): string => {
  switch (error.code) {
    case 'NETWORK_ERROR':
      return 'Unable to connect to the server. Please check your internet connection.';
    case 'SERVER_ERROR':
      return 'The service is temporarily unavailable. Please try again later.';
    case 'REQUEST_ERROR':
      return `Request failed: ${error.message}`;
    case 'VALIDATION_ERROR':
      return `Invalid input: ${error.message}`;
    default:
      return `An error occurred: ${error.message}`;
  }
};

// Log error for debugging purposes
export const logError = (error: Error, context?: string): void => {
  console.error(`[Chat Error${context ? ` - ${context}` : ''}]`, {
    name: error.name,
    message: error.message,
    stack: error.stack,
    timestamp: new Date().toISOString(),
  });
};
