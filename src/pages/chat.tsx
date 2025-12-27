import React, { useState, useRef, useEffect, useCallback, FormEvent } from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useBaseUrl from '@docusaurus/useBaseUrl';
import ReactMarkdown from 'react-markdown';
import {
  DOC_CHAPTERS,
  CHAPTER_TITLES,
  DEFAULT_CHAPTER,
} from '../constants/chapters';
import { useAuth } from '../components/Auth/AuthProvider';
import { AuthModal } from '../components/Auth/AuthModal';
import { SessionSidebar } from '../components/Chat/SessionSidebar';
import { getSessionMessages, sendMessage as sendChatMessage, ChatMessage as ChatMessageType } from '../lib/chat-sessions';
import { getSessionToken, signOut } from '../lib/auth-client';
import styles from './chat.module.css';

/**
 * Message type for chat interface
 */
interface Message {
  id: string;
  sender: 'user' | 'fubuni';
  content: string;
  timestamp: Date;
  navigatedTo?: string;
}

/**
 * Backend response structure
 */
interface BackendResponse {
  response: string;
  chapter?: string;
  section?: string;
  should_navigate?: boolean;
}

/**
 * Generate unique message ID
 */
function generateId(): string {
  return `msg-${Date.now()}-${Math.random().toString(36).substring(2, 9)}`;
}

/**
 * Get the API base URL based on environment
 */
function getApiBaseUrl(): string {
  if (typeof window === 'undefined') {
    return 'https://maryanrar-fubuni-chat-api.hf.space';
  }
  return window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://maryanrar-fubuni-chat-api.hf.space';
}

/**
 * Send icon SVG component
 */
function SendIcon({ className }: { className?: string }) {
  return (
    <svg
      className={className}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <line x1="22" y1="2" x2="11" y2="13" />
      <polygon points="22 2 15 22 11 13 2 9 22 2" />
    </svg>
  );
}

/**
 * Typing indicator component
 */
function TypingIndicator() {
  return (
    <div className={styles.typingIndicator} aria-label="Fubuni is typing">
      <span className={styles.typingDot} />
      <span className={styles.typingDot} />
      <span className={styles.typingDot} />
    </div>
  );
}

/**
 * Welcome message component shown when chat is empty
 */
function WelcomeMessage() {
  return (
    <div className={styles.welcomeMessage}>
      <div className={styles.welcomeIcon} role="img" aria-label="Fubuni">
        𝄞⨾𓍢ִ໋
      </div>
      <h2 className={styles.welcomeTitle}>Chat with Fubuni</h2>
      <p className={styles.welcomeText}>
        Ask me anything about humanoid robotics! I can help explain concepts and
        navigate you to relevant documentation.
      </p>
    </div>
  );
}

/**
 * Format time for display
 */
function formatTime(date: Date): string {
  return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
}

/**
 * Main chat content component (client-side only)
 */
function ChatContent() {
  const baseUrl = useBaseUrl('/');
  const { isAuthenticated, isLoading: authLoading, user } = useAuth();

  // Auth modal state - initialize based on localStorage to prevent flash
  const [showAuthModal, setShowAuthModal] = useState<boolean>(() => {
    // Check localStorage immediately on mount
    if (typeof window !== 'undefined') {
      const hasAuthUser = localStorage.getItem('fubuni_auth_user') !== null;
      return !hasAuthUser; // Show modal only if no auth user in localStorage
    }
    return false; // Default to hidden during SSR
  });

  // Chat state
  const [currentChapter, setCurrentChapter] = useState<string>(DEFAULT_CHAPTER);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [inputValue, setInputValue] = useState<string>('');
  const [error, setError] = useState<string | null>(null);
  const [iframeLoading, setIframeLoading] = useState<boolean>(true);

  // Session state
  const [activeSessionId, setActiveSessionId] = useState<string | null>(null);
  const [sidebarVisible, setSidebarVisible] = useState<boolean>(true);

  // Docs panel visibility - starts hidden, slides in on first navigation
  const [docsVisible, setDocsVisible] = useState<boolean>(false);
  const [hasNavigatedOnce, setHasNavigatedOnce] = useState<boolean>(false);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Show auth modal only when definitively not authenticated
  useEffect(() => {
    // Wait for auth to finish loading
    if (authLoading) {
      return;
    }

    // If authenticated via Better Auth, ensure modal is closed and localStorage is set
    if (isAuthenticated && user) {
      if (showAuthModal) {
        setShowAuthModal(false);
      }
      // Ensure localStorage is synced
      if (typeof window !== 'undefined') {
        const stored = localStorage.getItem('fubuni_auth_user');
        if (!stored) {
          localStorage.setItem('fubuni_auth_user', JSON.stringify(user));
          localStorage.setItem('fubuni_auth_timestamp', Date.now().toString());
        }
      }
      return;
    }

    // Check localStorage as primary source of truth
    const hasLocalStorage = typeof window !== 'undefined' &&
      localStorage.getItem('fubuni_auth_user') !== null;

    if (hasLocalStorage) {
      // Have auth data in localStorage, hide modal
      if (showAuthModal) {
        setShowAuthModal(false);
      }
    } else {
      // No auth data anywhere, show modal
      if (!showAuthModal && !isAuthenticated && !user) {
        setShowAuthModal(true);
      }
    }
  }, [authLoading, isAuthenticated, user, showAuthModal]);

  // Auto-close modal when authentication succeeds
  useEffect(() => {
    if (!authLoading && isAuthenticated && showAuthModal) {
      setShowAuthModal(false);
    }
  }, [authLoading, isAuthenticated, showAuthModal]);

  // Scroll to bottom when new messages arrive
  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading, scrollToBottom]);

  // Focus input on mount
  useEffect(() => {
    inputRef.current?.focus();
  }, []);

  /**
   * Get full documentation URL for a chapter
   */
  const getDocUrl = useCallback(
    (chapter: string): string => {
      const path = DOC_CHAPTERS[chapter] || DOC_CHAPTERS[DEFAULT_CHAPTER];
      // Build the full URL with baseUrl
      return `${baseUrl.replace(/\/$/, '')}${path}`;
    },
    [baseUrl]
  );

  /**
   * Handle chapter navigation from chapter buttons
   */
  const handleChapterClick = useCallback((chapter: string) => {
    setCurrentChapter(chapter);
    setIframeLoading(true);
  }, []);

  /**
   * Handle closing/minimizing the docs panel
   */
  const handleCloseDocs = useCallback(() => {
    setDocsVisible(false);
  }, []);

  /**
   * Handle opening the docs panel manually
   */
  const handleOpenDocs = useCallback(() => {
    setDocsVisible(true);
    setIframeLoading(true);
  }, []);

  /**
   * Load messages for a session
   */
  const loadSessionMessages = useCallback(async (sessionId: string) => {
    try {
      const data = await getSessionMessages(sessionId);
      const loadedMessages: Message[] = data.messages.map((msg: ChatMessageType) => ({
        id: msg.id,
        sender: msg.sender as 'user' | 'fubuni',
        content: msg.content,
        timestamp: new Date(msg.timestamp),
      }));
      setMessages(loadedMessages);
      setActiveSessionId(sessionId);
    } catch (err) {
      console.error('Failed to load session messages:', err);
      setError('Failed to load chat history');
    }
  }, []);

  /**
   * Handle session selection from sidebar
   */
  const handleSessionSelect = useCallback((sessionId: string) => {
    if (sessionId === activeSessionId) return;
    loadSessionMessages(sessionId);
  }, [activeSessionId, loadSessionMessages]);

  /**
   * Handle new session creation
   */
  const handleNewSession = useCallback((sessionId: string) => {
    setMessages([]);
    setActiveSessionId(sessionId || null);
    setError(null);
  }, []);

  /**
   * Handle sign out
   */
  const handleSignOut = useCallback(async () => {
    try {
      // Clear persisted auth first (localStorage)
      if (typeof window !== 'undefined') {
        localStorage.removeItem('fubuni_auth_user');
      }
      await signOut();
      setMessages([]);
      setActiveSessionId(null);
      setShowAuthModal(true);
    } catch (err) {
      console.error('Sign out error:', err);
    }
  }, []);

  /**
   * Handle agent response and auto-navigate if needed
   */
  const handleAgentResponse = useCallback(
    (response: BackendResponse, messageId: string) => {
      if (response.should_navigate && response.chapter) {
        const targetChapter = response.chapter;

        // Validate the chapter exists
        if (DOC_CHAPTERS[targetChapter]) {
          setCurrentChapter(targetChapter);
          setIframeLoading(true);

          // First navigation - trigger slide-in animation
          if (!hasNavigatedOnce) {
            setHasNavigatedOnce(true);
          }

          // Show docs panel (slides in from left)
          setDocsVisible(true);

          // Update the message to show navigation indicator
          setMessages((prev) =>
            prev.map((msg) =>
              msg.id === messageId
                ? { ...msg, navigatedTo: targetChapter }
                : msg
            )
          );
        }
      }
    },
    [hasNavigatedOnce]
  );

  // Store pending message for re-auth scenarios
  const [pendingMessage, setPendingMessage] = useState<string | null>(null);

  /**
   * Send message to backend
   */
  const sendMessage = useCallback(
    async (messageText: string) => {
      if (!messageText.trim() || isLoading) return;

      // Check authentication - preserve message if auth required
      if (!isAuthenticated) {
        setPendingMessage(messageText.trim());
        setShowAuthModal(true);
        return;
      }

      // Clear any previous error and pending message
      setError(null);
      setPendingMessage(null);

      // Add user message
      const userMessage: Message = {
        id: generateId(),
        sender: 'user',
        content: messageText.trim(),
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, userMessage]);
      setInputValue('');
      setIsLoading(true);

      try {
        // Use the authenticated chat API
        const data = await sendChatMessage(messageText.trim(), activeSessionId || undefined);

        // Update session ID if a new one was created
        if (data.session_id && data.session_id !== activeSessionId) {
          setActiveSessionId(data.session_id);
          // Refresh the sidebar to show the new session
          if ((window as unknown as Record<string, unknown>).__refreshSessions) {
            ((window as unknown as Record<string, unknown>).__refreshSessions as () => void)();
          }
        }

        // Add bot response
        const botMessageId = generateId();
        const botMessage: Message = {
          id: botMessageId,
          sender: 'fubuni',
          content: data.response,
          timestamp: new Date(),
        };
        setMessages((prev) => [...prev, botMessage]);

        // Handle navigation if needed
        handleAgentResponse(data as BackendResponse, botMessageId);
      } catch (err) {
        console.error('Chat error:', err);
        // Check if it's an auth error (401)
        const errorMessage = err instanceof Error ? err.message : 'Failed to send message';
        if (errorMessage.includes('401') || errorMessage.toLowerCase().includes('unauthorized') || errorMessage.toLowerCase().includes('not authenticated')) {
          // Preserve message and prompt re-auth
          setPendingMessage(messageText.trim());
          setShowAuthModal(true);
          setError('Session expired. Please sign in again.');
        } else {
          setError(errorMessage);
        }
      } finally {
        setIsLoading(false);
        // Refocus input after response
        inputRef.current?.focus();
      }
    },
    [isLoading, isAuthenticated, activeSessionId, handleAgentResponse]
  );

  // Restore pending message to input after successful re-auth
  useEffect(() => {
    if (isAuthenticated && pendingMessage && !showAuthModal) {
      setInputValue(pendingMessage);
      setPendingMessage(null);
      inputRef.current?.focus();
    }
  }, [isAuthenticated, pendingMessage, showAuthModal]);

  /**
   * Handle form submission
   */
  const handleSubmit = useCallback(
    (e: FormEvent) => {
      e.preventDefault();
      sendMessage(inputValue);
    },
    [inputValue, sendMessage]
  );

  /**
   * Handle input key press (Enter to send, Shift+Enter for newline)
   */
  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        sendMessage(inputValue);
      }
    },
    [inputValue, sendMessage]
  );

  /**
   * Handle iframe load complete
   */
  const handleIframeLoad = useCallback(() => {
    setIframeLoading(false);
  }, []);

  /**
   * Auto-resize textarea based on content
   */
  const handleInputChange = useCallback(
    (e: React.ChangeEvent<HTMLTextAreaElement>) => {
      setInputValue(e.target.value);
      // Reset height to auto to get the correct scrollHeight
      e.target.style.height = 'auto';
      // Set height to scrollHeight, but max out at 120px
      e.target.style.height = `${Math.min(e.target.scrollHeight, 120)}px`;
    },
    []
  );

  // Show loading state while checking auth
  if (authLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.chatPanel}>
          <div className={styles.messagesContainer}>
            <div className={styles.loading}>Loading...</div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <>
      {/* Auth Modal */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => {
          // Allow closing only if authenticated
          if (isAuthenticated) {
            setShowAuthModal(false);
          }
        }}
      />

      <div className={`${styles.container} ${isAuthenticated ? styles.withSidebar : ''} ${docsVisible ? styles.withDocs : styles.chatOnly}`}>
        {/* Session Sidebar */}
        {isAuthenticated && (
          <SessionSidebar
            activeSessionId={activeSessionId}
            onSessionSelect={handleSessionSelect}
            onNewSession={handleNewSession}
            isAuthenticated={isAuthenticated}
          />
        )}

        {/* Documentation viewer panel - slides in from left */}
        <div className={styles.docViewer}>
          {/* Close button */}
          <button
            type="button"
            className={styles.closeDocsButton}
            onClick={handleCloseDocs}
            aria-label="Close documentation panel"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>

          {/* Chapter navigation */}
          <nav className={styles.chapterNav} aria-label="Chapter navigation">
            {Object.entries(CHAPTER_TITLES).map(([chapterId, title]) => (
              <button
                key={chapterId}
                type="button"
                className={`${styles.chapterButton} ${
                  currentChapter === chapterId ? styles.chapterButtonActive : ''
                }`}
                onClick={() => handleChapterClick(chapterId)}
                aria-pressed={currentChapter === chapterId}
              >
                {title}
              </button>
            ))}
          </nav>

          {/* Documentation iframe */}
          <iframe
            src={getDocUrl(currentChapter)}
            className={`${styles.iframe} ${iframeLoading ? styles.iframeLoading : ''}`}
            title={`Documentation: ${CHAPTER_TITLES[currentChapter] || 'Robotics Course'}`}
            onLoad={handleIframeLoad}
          />
        </div>

        {/* Chat panel */}
        <div className={styles.chatPanel}>
          {/* Chat header */}
          <header className={styles.chatHeader}>
            <div className={styles.chatAvatar} role="img" aria-label="Fubuni">
              𝄞⨾𓍢ִ໋
            </div>
            <div className={styles.chatHeaderInfo}>
              <h1 className={styles.chatTitle}>.𖥔 ݁FUBUNI .˖</h1>
              <p className={styles.chatSubtitle}>
                {isAuthenticated && user?.name
                  ? `Hi, ${user.name}!`
                  : 'Robotics Learning Assistant'}
              </p>
            </div>
            <div className={styles.headerButtons}>
              {/* Show docs button when docs are hidden */}
              {!docsVisible && (
                <button
                  type="button"
                  className={styles.showDocsButton}
                  onClick={handleOpenDocs}
                  aria-label="Show documentation"
                >
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20" />
                    <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z" />
                  </svg>
                  <span>Docs</span>
                </button>
              )}
              {/* Sign out button */}
              {isAuthenticated && (
                <button
                  type="button"
                  className={styles.signOutButton}
                  onClick={handleSignOut}
                  aria-label="Sign out"
                >
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4" />
                    <polyline points="16 17 21 12 16 7" />
                    <line x1="21" y1="12" x2="9" y2="12" />
                  </svg>
                </button>
              )}
            </div>
          </header>

          {/* Messages container */}
          <div
            className={styles.messagesContainer}
            role="log"
            aria-label="Chat messages"
            aria-live="polite"
          >
            {messages.length === 0 && !isLoading ? (
              <WelcomeMessage />
            ) : (
              <>
                {messages.map((message) => (
                  <div
                    key={message.id}
                    className={`${styles.message} ${
                      message.sender === 'user'
                        ? styles.userMessage
                        : styles.botMessage
                    }`}
                  >
                    <div className={styles.messageBubble}>
                      <ReactMarkdown>{message.content}</ReactMarkdown>
                    </div>
                    {message.navigatedTo && (
                      <div className={styles.navigationIndicator}>
                        <span
                          className={styles.navigationIcon}
                          role="img"
                          aria-label="Navigation"
                        >
                          📍
                        </span>
                        <span>
                          Navigated to:{' '}
                          {CHAPTER_TITLES[message.navigatedTo] ||
                            message.navigatedTo}
                        </span>
                      </div>
                    )}
                    <span className={styles.messageTime}>
                      {formatTime(message.timestamp)}
                    </span>
                  </div>
                ))}
                {isLoading && (
                  <div className={`${styles.message} ${styles.botMessage}`}>
                    <TypingIndicator />
                  </div>
                )}
              </>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Error message */}
          {error && (
            <div className={styles.errorMessage} role="alert">
              <span role="img" aria-label="Error">
                ⚠️
              </span>
              <span>{error}</span>
            </div>
          )}

          {/* Input container */}
          <div className={styles.inputContainer}>
            <form className={styles.inputForm} onSubmit={handleSubmit}>
              <textarea
                ref={inputRef}
                className={styles.chatInput}
                value={inputValue}
                onChange={handleInputChange}
                onKeyDown={handleKeyDown}
                placeholder={isAuthenticated ? "Ask about robotics..." : "Sign in to chat..."}
                disabled={isLoading || !isAuthenticated}
                rows={1}
                aria-label="Message input"
              />
              <button
                type="submit"
                className={styles.sendButton}
                disabled={isLoading || !inputValue.trim() || !isAuthenticated}
                aria-label="Send message"
              >
                <SendIcon className={styles.sendIcon} />
              </button>
            </form>
          </div>
        </div>
      </div>
    </>
  );
}

/**
 * Chat page component with SSR protection
 */
export default function ChatPage(): React.ReactNode {
  return (
    <Layout
      title="Chat with Fubuni"
      description="Interactive chat assistant for learning humanoid robotics"
      noFooter
      wrapperClassName="chat-page"
    >
      <BrowserOnly fallback={<div>Loading chat interface...</div>}>
        {() => <ChatContent />}
      </BrowserOnly>
    </Layout>
  );
}
