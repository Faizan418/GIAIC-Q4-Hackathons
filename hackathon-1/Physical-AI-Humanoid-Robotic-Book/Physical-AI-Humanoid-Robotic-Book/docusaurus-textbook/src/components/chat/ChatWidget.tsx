import React, { useState, useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import { ChatMessage as ChatMessageType } from '../../types/chat';
import { useChatApi } from '../../services/chat-api';
import SessionManager from '../../utils/session';
import { handleApiError, displayErrorMessage, logError } from '../../utils/error-handler';
import { getTextSelection, addTextSelectionListener, TextSelectionResult } from '../../utils/text-selection';

interface ChatWidgetProps {
  initialOpen?: boolean;
}

const ChatWidget: React.FC<ChatWidgetProps> = ({ initialOpen = false }) => {
  const chatApi = useChatApi();
  const [isOpen, setIsOpen] = useState(initialOpen);
  const [messages, setMessages] = useState<ChatMessageType[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<TextSelectionResult | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Load messages from session on component mount
  useEffect(() => {
    setMessages(SessionManager.getMessages());
  }, []);

  // Add text selection listener when component mounts
  useEffect(() => {
    const cleanup = addTextSelectionListener((selection) => {
      if (selection) {
        setSelectedText(selection);
      }
    });

    return cleanup;
  }, []);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async (content: string) => {
    if (isLoading) return;

    try {
      setIsLoading(true);
      setError(null);

      // Add user message to UI immediately
      const userMessage = SessionManager.addMessage({
        sender: 'user',
        content: content,
        status: 'pending',
      });

      setMessages([...SessionManager.getMessages()]);

      // Prepare the API request
      const conversationId = SessionManager.getConversationId();
      const request = {
        query: content,
        conversation_id: conversationId || undefined,
        context: {
          page_url: window.location.pathname,
          previous_messages: SessionManager.getMessages()
            .filter(msg => msg.id !== userMessage.id) // Exclude the current message
            .map(msg => ({
              role: msg.sender,
              content: msg.content,
            })),
        },
      };

      // Call the API
      const response = await chatApi.query(request);

      // Update user message status to delivered
      SessionManager.updateMessageStatus(userMessage.id, 'delivered');

      // Add assistant response
      const assistantMessage = SessionManager.addMessage({
        sender: 'assistant',
        content: response.answer,
        status: 'delivered',
      });

      // Update conversation ID if it was returned
      if (response.conversation_id && response.conversation_id !== conversationId) {
        // Note: In a real implementation, we'd update the session with the new conversation ID
        // For now, we'll just log it
        console.log('New conversation ID:', response.conversation_id);
      }

      setMessages([...SessionManager.getMessages()]);
    } catch (err) {
      const chatError = handleApiError(err);
      setError(displayErrorMessage(chatError));
      logError(chatError, 'ChatWidget.handleSendMessage');

      // Update the user message status to error
      const allMessages = SessionManager.getMessages();
      const lastUserMessage = [...allMessages].reverse().find(msg =>
        msg.sender === 'user' && msg.status === 'pending'
      );

      if (lastUserMessage) {
        SessionManager.updateMessageStatus(lastUserMessage.id, 'error');
        setMessages([...SessionManager.getMessages()]);
      }
    } finally {
      setIsLoading(false);
    }
  };

  const handleSendSelectedText = async () => {
    if (!selectedText || isLoading) return;

    try {
      setIsLoading(true);
      setError(null);

      // Add user message for selected text to UI immediately
      const userMessage = SessionManager.addMessage({
        sender: 'user',
        content: `Explain this: "${selectedText.content}"`,
        status: 'pending',
      });

      setMessages([...SessionManager.getMessages()]);

      // Prepare the API request for text selection
      const conversationId = SessionManager.getConversationId();
      const request = {
        selected_text: selectedText.content,
        context: {
          page_url: selectedText.sourceUrl,
          surrounding_text: selectedText.context,
          conversation_id: conversationId || undefined,
        },
      };

      // Call the text selection API
      const response = await chatApi.textSelectionQuery(request);

      // Update user message status to delivered
      SessionManager.updateMessageStatus(userMessage.id, 'delivered');

      // Add assistant response
      const assistantMessage = SessionManager.addMessage({
        sender: 'assistant',
        content: response.answer,
        status: 'delivered',
      });

      setMessages([...SessionManager.getMessages()]);
    } catch (err) {
      const chatError = handleApiError(err);
      setError(displayErrorMessage(chatError));
      logError(chatError, 'ChatWidget.handleSendSelectedText');

      // Update the user message status to error
      const allMessages = SessionManager.getMessages();
      const lastUserMessage = [...allMessages].reverse().find(msg =>
        msg.sender === 'user' && msg.status === 'pending'
      );

      if (lastUserMessage) {
        SessionManager.updateMessageStatus(lastUserMessage.id, 'error');
        setMessages([...SessionManager.getMessages()]);
      }
    } finally {
      setIsLoading(false);
      setSelectedText(null);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  if (!isOpen) {
    return (
      <button
        className="chat-toggle-button"
        onClick={toggleChat}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#4f6fef',
          color: 'white',
          border: 'none',
          fontSize: '24px',
          cursor: 'pointer',
          zIndex: 1000,
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
        }}
        aria-label="Open chat"
      >
        💬
      </button>
    );
  }

  return (
    <div className="chat-widget">
      <div className="chat-header">
        <span>AI Textbook Assistant</span>
        <button className="close-btn" onClick={closeChat} aria-label="Close chat">
          ×
        </button>
      </div>

      {error && (
        <div className="error-message">
          {error}
        </div>
      )}

      <div className="chat-messages">
        {messages.map((message) => (
          <ChatMessage key={message.id} message={message} />
        ))}
        {isLoading && (
          <div className="loading-indicator">
            <span>AI is thinking</span>
            <div className="loading-dots">
              <div className="loading-dot"></div>
              <div className="loading-dot"></div>
              <div className="loading-dot"></div>
            </div>
          </div>
        )}
        {selectedText && (
          <div style={{
            padding: '10px',
            backgroundColor: '#e3f2fd',
            borderRadius: '4px',
            marginBottom: '10px',
            fontSize: '14px'
          }}>
            Selected: "{selectedText.content.substring(0, 60)}{selectedText.content.length > 60 ? '...' : ''}"
            <button
              onClick={handleSendSelectedText}
              disabled={isLoading}
              style={{
                marginLeft: '10px',
                padding: '4px 8px',
                fontSize: '12px',
                backgroundColor: '#2196f3',
                color: 'white',
                border: 'none',
                borderRadius: '3px',
                cursor: 'pointer'
              }}
            >
              Ask about this
            </button>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <ChatInput
        onSendMessage={handleSendMessage}
        disabled={isLoading}
      />
    </div>
  );
};

export default ChatWidget;
