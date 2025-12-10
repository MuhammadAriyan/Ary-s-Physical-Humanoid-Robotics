import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import FubuniBubble from './FubuniBubble';
import ChatDrawer from './ChatDrawer';
import ChatModal from './ChatModal';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';

interface Message {
  id: string;
  sender: 'user' | 'fubuni';
  content: string;
  timestamp: Date;
}

interface FubuniChatProps {
  backendUrl?: string;
}

const FubuniChat: React.FC<FubuniChatProps> = ({ backendUrl = 'http://localhost:8000' }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage: Message = {
      id: Date.now().toString(),
      sender: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Send message to backend
      const response = await fetch(`${backendUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          session_id: sessionId || undefined,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      
      // Create a new Fubuni message
      const fubuniMessage: Message = {
        id: `fubuni-${Date.now()}`,
        sender: 'fubuni',
        content: data.response,
        timestamp: new Date(),
      };

      // Add message to display
      setMessages(prev => [...prev, fubuniMessage]);

      // Update session ID if it was returned
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
      }
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to chat
      const errorMessage: Message = {
        id: `error-${Date.now()}`,
        sender: 'fubuni',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const toggleFullscreen = () => {
    setIsFullscreen(!isFullscreen);
  };

  const minimizeFullscreen = () => {
    setIsFullscreen(false);
  };

  const renderMessages = () => (
    <>
      {messages.map((message) => (
        <ChatMessage
          key={message.id}
          sender={message.sender}
          content={message.content}
        />
      ))}
      <div ref={messagesEndRef} />
    </>
  );

  return (
    <div className={styles['fubuni-chat-container']}>
      {isOpen && !isFullscreen && (
        <ChatDrawer
          isOpen={isOpen}
          onClose={toggleChat}
          onExpand={toggleFullscreen}
        >
          {renderMessages()}
          <ChatInput
            value={inputValue}
            onChange={setInputValue}
            onSubmit={handleSubmit}
            isLoading={isLoading}
          />
        </ChatDrawer>
      )}

      {isFullscreen && (
        <ChatModal
          isOpen={isFullscreen}
          onClose={toggleChat}
          onMinimize={minimizeFullscreen}
        >
          {renderMessages()}
          <ChatInput
            value={inputValue}
            onChange={setInputValue}
            onSubmit={handleSubmit}
            isLoading={isLoading}
          />
        </ChatModal>
      )}

      {!isOpen && !isFullscreen && (
        <FubuniBubble onClick={toggleChat} />
      )}
    </div>
  );
};

export default FubuniChat;