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
 * Web search result from DuckDuckGo
 */
interface WebSearchResult {
  title: string;
  url: string;
  snippet: string;
  favicon?: string;
}

/**
 * Backend response structure
 */
interface BackendResponse {
  response: string;
  chapter?: string;
  section?: string;
  should_navigate?: boolean;
  web_sources?: WebSearchResult[];
  used_web_search?: boolean;
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
 * Typing indicator component with optional status text
 */
function TypingIndicator({ status }: { status?: string }) {
  return (
    <div className={styles.typingIndicator} aria-label={status || "Fubuni is typing"}>
      {status && <span className={styles.typingStatus}>{status}</span>}
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
 * Web Sources Panel component - slides in from right
 */
function WebSourcesPanel({
  sources,
  onClose,
}: {
  sources: WebSearchResult[];
  onClose: () => void;
}) {
  return (
    <div className={styles.webSourcesPanel}>
      {/* Close button */}
      <button
        type="button"
        className={styles.closeSourcesButton}
        onClick={onClose}
        aria-label="Close web sources panel"
      >
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <line x1="18" y1="6" x2="6" y2="18" />
          <line x1="6" y1="6" x2="18" y2="18" />
        </svg>
      </button>

      <div className={styles.sourcesHeader}>
        <span className={styles.sourcesIcon} role="img" aria-label="Web sources">
          🌐
        </span>
        <h3 className={styles.sourcesTitle}>Web Sources</h3>
      </div>

      <div className={styles.sourcesList}>
        {sources.map((source, index) => (
          <a
            key={index}
            href={source.url}
            target="_blank"
            rel="noopener noreferrer"
            className={styles.sourceItem}
          >
            <div className={styles.sourceTitleRow}>
              {source.favicon && (
                <img
                  src={source.favicon}
                  alt=""
                  className={styles.sourceFavicon}
                  onError={(e) => {
                    e.currentTarget.style.display = 'none';
                  }}
                />
              )}
              <h4 className={styles.sourceTitle}>{source.title}</h4>
            </div>
            <p className={styles.sourceSnippet}>{source.snippet}</p>
            <span className={styles.sourceUrl}>{source.url}</span>
          </a>
        ))}
      </div>
    </div>
  );
}

/**
 * Main chat content component (client-side only)
 */
function ChatContent() {
  const baseUrl = useBaseUrl('/');
  const [currentChapter, setCurrentChapter] = useState<string>(DEFAULT_CHAPTER);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [inputValue, setInputValue] = useState<string>('');
  const [error, setError] = useState<string | null>(null);
  const [iframeLoading, setIframeLoading] = useState<boolean>(true);

  // Docs panel visibility - starts hidden, slides in on first navigation
  const [docsVisible, setDocsVisible] = useState<boolean>(false);
  const [hasNavigatedOnce, setHasNavigatedOnce] = useState<boolean>(false);

  // Web sources panel visibility and data
  const [sourcesVisible, setSourcesVisible] = useState<boolean>(false);
  const [webSources, setWebSources] = useState<WebSearchResult[]>([]);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

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
   * Handle closing the web sources panel
   */
  const handleCloseSources = useCallback(() => {
    setSourcesVisible(false);
  }, []);

  /**
   * Handle agent response and auto-navigate if needed
   */
  const handleAgentResponse = useCallback(
    (response: BackendResponse, messageId: string) => {
      // Handle documentation navigation
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

      // Handle web sources - T023: Only show panel if there are sources
      if (response.used_web_search && response.web_sources && response.web_sources.length > 0) {
        setWebSources(response.web_sources);
        setSourcesVisible(true);
      }
    },
    [hasNavigatedOnce]
  );

  /**
   * Send message to backend
   */
  const sendMessage = useCallback(
    async (messageText: string) => {
      if (!messageText.trim() || isLoading) return;

      // Clear any previous error
      setError(null);

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
        const apiUrl = getApiBaseUrl();
        const response = await fetch(`${apiUrl}/api/chat`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            message: messageText.trim(),
            stream: false,
          }),
        });

        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }

        const data: BackendResponse = await response.json();

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
        handleAgentResponse(data, botMessageId);
      } catch (err) {
        console.error('Chat error:', err);
        setError(
          err instanceof Error
            ? err.message
            : 'Failed to send message. Please try again.'
        );
      } finally {
        setIsLoading(false);
        // Refocus input after response
        inputRef.current?.focus();
      }
    },
    [isLoading, handleAgentResponse]
  );

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

  // Determine layout class based on which panels are visible
  const getLayoutClass = () => {
    if (docsVisible && sourcesVisible) return styles.withAll;
    if (docsVisible) return styles.withDocs;
    if (sourcesVisible) return styles.withSources;
    return styles.chatOnly;
  };

  return (
    <div className={`${styles.container} ${getLayoutClass()}`}>
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
            <p className={styles.chatSubtitle}>Robotics Learning Assistant</p>
          </div>
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
              placeholder="Ask about robotics..."
              disabled={isLoading}
              rows={1}
              aria-label="Message input"
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send message"
            >
              <SendIcon className={styles.sendIcon} />
            </button>
          </form>
        </div>
      </div>

      {/* Web Sources Panel - slides in from right */}
      {webSources.length > 0 && (
        <WebSourcesPanel sources={webSources} onClose={handleCloseSources} />
      )}
    </div>
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
