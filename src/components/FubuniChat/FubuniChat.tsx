import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import FubuniBubble from './FubuniBubble';
import ChatDrawer from './ChatDrawer';
import ChatModal from './ChatModal';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import { sendMessage } from '../../lib/chat-sessions';

interface Message {
  id: string;
  sender: 'user' | 'fubuni';
  content: string;
  timestamp: Date;
}

interface FubuniChatProps {}

const FubuniChat: React.FC<FubuniChatProps> = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Add welcome message when chat opens for the first time
  useEffect(() => {
    if (isOpen && messages.length === 0) {
      const welcomeMessage: Message = {
        id: 'welcome',
        sender: 'fubuni',
        content: 'Hi! I\'m Fubuni, your robotics learning assistant. Ask me anything about Physical & Humanoid Robotics!',
        timestamp: new Date(),
      };
      setMessages([welcomeMessage]);
    }
  }, [isOpen, messages.length]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessageContent = inputValue;

    // Add user message to the chat
    const userMessage: Message = {
      id: Date.now().toString(),
      sender: 'user',
      content: userMessageContent,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Send message using the authenticated chat-sessions API
      const data = await sendMessage(userMessageContent);

      // Create a new Fubuni message
      const fubuniMessage: Message = {
        id: `fubuni-${Date.now()}`,
        sender: 'fubuni',
        content: data.response,
        timestamp: new Date(),
      };

      // Add message to display
      setMessages(prev => [...prev, fubuniMessage]);

    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to chat
      const errorMessage: Message = {
        id: `error-${Date.now()}`,
        sender: 'fubuni',
        content: 'Sorry, I encountered an error. The chat service might be unavailable. Please try again later.',
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

  const clearChat = () => {
    setMessages([]);
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
      {isLoading && (
        <div className={styles['fubuni-message'] + ' ' + styles['fubuni']}>
          <p className={styles['fubuni-message-content']}>
            <span className={styles['typing-indicator']}>Thinking...</span>
          </p>
        </div>
      )}
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
          onSignOut={messages.length > 1 ? clearChat : undefined}
          userName={messages.length > 1 ? "Clear Chat" : undefined}
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
          onSignOut={messages.length > 1 ? clearChat : undefined}
          userName={messages.length > 1 ? "Clear Chat" : undefined}
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
