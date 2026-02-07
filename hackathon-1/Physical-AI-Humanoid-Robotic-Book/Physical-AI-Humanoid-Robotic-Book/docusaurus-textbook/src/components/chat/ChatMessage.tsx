import React from 'react';
import { ChatMessage as ChatMessageType } from '../../types/chat';

interface ChatMessageProps {
  message: ChatMessageType;
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const isUser = message.sender === 'user';
  const messageClass = `message ${isUser ? 'user' : 'assistant'}`;

  // Format timestamp
  const formatTime = (date: Date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className={messageClass}>
      <div className="content">{message.content}</div>
      <div className="timestamp">{formatTime(message.timestamp)}</div>
    </div>
  );
};

export default ChatMessage;
