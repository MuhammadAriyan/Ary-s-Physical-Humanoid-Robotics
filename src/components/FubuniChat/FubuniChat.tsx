import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import FubuniBubble from './FubuniBubble';
import ChatDrawer from './ChatDrawer';
import ChatModal from './ChatModal';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import { AuthModal, useAuth } from '../Auth';
import { getSession, signOut } from '../../lib/auth-client';

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
  const { isAuthenticated, isLoading: authLoading, user, refetch } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState(false);
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

    // Check if user is authenticated before sending
    if (!isAuthenticated) {
      setShowAuthModal(true);
      return;
    }

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
      // Get current session token for Authorization header
      const session = await getSession();
      const token = session?.data?.session?.token;

      // Build headers with Authorization if token available
      const headers: HeadersInit = {
        'Content-Type': 'application/json',
      };
      if (token) {
        headers['Authorization'] = `Bearer ${token}`;
      }

      // Send message to backend
      const response = await fetch(`${backendUrl}/api/chat`, {
        method: 'POST',
        headers,
        body: JSON.stringify({
          message: inputValue,
          session_id: sessionId || undefined,
        }),
      });

      // Handle 401 Unauthorized - show auth modal
      if (response.status === 401) {
        setShowAuthModal(true);
        const errorMessage: Message = {
          id: `error-${Date.now()}`,
          sender: 'fubuni',
          content: 'Your session has expired. Please sign in again to continue.',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
        return;
      }

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
    // Show auth modal if not authenticated when trying to open chat
    if (!isOpen && !isAuthenticated && !authLoading) {
      setShowAuthModal(true);
      return;
    }
    setIsOpen(!isOpen);
  };

  const handleAuthSuccess = () => {
    refetch();
    setShowAuthModal(false);
    // Open chat after successful authentication
    setIsOpen(true);
  };

  const handleSignOut = async () => {
    try {
      await signOut();
      // Clear local state
      setMessages([]);
      setSessionId(null);
      setIsOpen(false);
      setIsFullscreen(false);
      // Refresh auth state
      refetch();
      // Show auth modal
      setShowAuthModal(true);
    } catch (error) {
      console.error('Sign out error:', error);
    }
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
      {/* Auth Modal */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={handleAuthSuccess}
      />

      {isOpen && !isFullscreen && (
        <ChatDrawer
          isOpen={isOpen}
          onClose={toggleChat}
          onExpand={toggleFullscreen}
          onSignOut={isAuthenticated ? handleSignOut : undefined}
          userName={user?.name || user?.email}
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
          onSignOut={isAuthenticated ? handleSignOut : undefined}
          userName={user?.name || user?.email}
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